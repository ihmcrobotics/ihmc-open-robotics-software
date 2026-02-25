#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define GLOBAL_CENTER_INDEX 2
#define LOCAL_CELLS_PER_AXIS 3
#define GLOBAL_CELLS_PER_AXIS 4
#define DEPTH_INPUT_HEIGHT 5
#define DEPTH_INPUT_WIDTH 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define DEPTH_FX 9
#define DEPTH_FY 10
#define MIN_HEIGHT_REGISTRATION 11
#define MAX_HEIGHT_REGISTRATION 12
#define MIN_CLAMP_HEIGHT 13
#define MAX_CLAMP_HEIGHT 14
#define KALMAN_FILTER_PREDICTION_NOISE 15
#define ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED 16
#define VARIANCE_PER_METER 17
#define VARIANCE_PER_TRANSLATION_SPEED 18
#define VARIANCE_PER_ROTATION_SPEED 19
#define GROUND_HEIGHT 20
#define MIN_DEPTH_TO_ACCEPT 21

__device__ float3 back_project_perspective(int2 pos, float Z, const float *params)
{
    float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
    float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
    float3 point = make_float3(Z, -X, -Y);
    return point;
}

/**
 * @brief Height map update KERNEL: One thread per depth pixel for peak optimization.
 * Reads in the depth for each pixel in the image.
 * Back projects the points into the Z-Up frame using transforms.
 * Atomically adds the points to the maps.
 */
extern "C"
__global__ void heightMapUpdateDataKernel(const unsigned short* __restrict__ depthImage, size_t pitchDepth,
                                          float* __restrict__ sumMap, size_t pitchSum,
                                          int* __restrict__ countMap, size_t pitchCount,
                                          float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                          float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                          const float* __restrict__ params,
                                          float* sensorToZUpFrameTf,
                                          float* sensorToWorldFrameTf,
                                          const float linearMotionMagnitude,
                                          const float angularMotionMagnitude)
{
    // Thread indices now correspond to pixel coordinates
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    const int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);
    const int cellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const float cellSize = params[CELL_SIZE];
    const int centerIndex = static_cast<int>(params[LOCAL_CENTER_INDEX]);
    const float varPerMeter = params[VARIANCE_PER_METER];
    const float varPerTranslationSpeed = params[VARIANCE_PER_TRANSLATION_SPEED];
    const float varPerRotationSpeed = params[VARIANCE_PER_ROTATION_SPEED];

    // Bounds check against depth image dimensions
    if (xIndex >= depthWidth || yIndex >= depthHeight)
        return;

    // Coalesced read from depth image
    const unsigned short* rowPtr = (const unsigned short*)((const char*)depthImage + yIndex * pitchDepth);
    float depth = rowPtr[xIndex] * 0.001f; // Scale to meters

    // Early exit for invalid depth
    if (depth < params[MIN_DEPTH_TO_ACCEPT])
        return;

    // Back-project and transform
    float3 queryPointInSensorFrame = back_project_perspective(make_int2(xIndex, yIndex), depth, params);
    float3 queryPointInZUpFrame = transformPoint3D(queryPointInSensorFrame, sensorToZUpFrameTf);

    // 1. Get the raw world position from the transform matrix
    float worldCamX = sensorToWorldFrameTf[3];
    float worldCamY = sensorToWorldFrameTf[7];

    // 2. Calculate the "Snapped" position (the nearest grid intersection)
    // Using floor ensures the grid lines stay at multiples of cellSize (0.0, 0.1, 0.2...)
    float snappedCenterX = (floorf(worldCamX / cellSize) * cellSize);
    float snappedCenterY = (floorf(worldCamY / cellSize) * cellSize);

    // 3. Calculate the sub-cell offset
    // This is how far the camera has "strayed" from the snapped grid point
    float subCellOffsetX = worldCamX - snappedCenterX;
    float subCellOffsetY = worldCamY - snappedCenterY;

    // 4. Apply the offset to the point relative to the sensor
    // This "stabilizes" the point against the camera's sub-pixel movement
    float2 stableXY;
    stableXY.x = queryPointInZUpFrame.x + subCellOffsetX;
    stableXY.y = queryPointInZUpFrame.y + subCellOffsetY;

    // Get the correct cell index for the maps
    float2 xyCoords = make_float2(queryPointInZUpFrame.x, queryPointInZUpFrame.y);
    int2 cellIndex = coordinate_to_indices(stableXY, make_float2(0.0f, 0.0f), cellSize, centerIndex);

    // Bounds check against the local map dimensions because we are scanning the entire depth image
    if (cellIndex.x < 0 || cellIndex.x >= cellsPerAxis || cellIndex.y < 0 || cellIndex.y >= cellsPerAxis)
        return;

    // Pointers to the target cell in each temporary map
    float* sumPtr = (float*)((char*)sumMap + cellIndex.y * pitchSum) + cellIndex.x;
    int* countPtr = (int*)((char*)countMap + cellIndex.y * pitchCount) + cellIndex.x;
    float* sumSqPtr = (float*)((char*)sumOfSquaresMap + cellIndex.y * pitchSumSq) + cellIndex.x;
    float* motionVarPtr = (float*)((char*)motionVarianceSumMap + cellIndex.y * pitchMotionVar) + cellIndex.x;

    // Atomically add this pixel's contribution
    atomicAdd(sumPtr, queryPointInZUpFrame.z);
    atomicAdd(sumSqPtr, queryPointInZUpFrame.z * queryPointInZUpFrame.z);
    atomicAdd(countPtr, 1);

    // Also calculate and add motion variance contribution
    float distance = sqrtf(queryPointInSensorFrame.x * queryPointInSensorFrame.x +
                           queryPointInSensorFrame.y * queryPointInSensorFrame.y +
                           queryPointInSensorFrame.z * queryPointInSensorFrame.z);
    float motionVarianceF = distance * varPerMeter + linearMotionMagnitude * varPerTranslationSpeed + angularMotionMagnitude * distance * varPerRotationSpeed;
    atomicAdd(motionVarPtr, motionVarianceF);
}

/**
 * @brief Compute Local Map KERNEL: The threads correspond to cell indices
 * Takes the maps that hold all the sums, and computes the mean, variance, and motion variance per cell
 */
extern "C"
__global__ void computeLocalMap(const float* __restrict__ sumMap, size_t pitchSum,
                                const int* __restrict__ countMap, size_t pitchCount,
                                const float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                const float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                float* __restrict__ localMeanMap, size_t pitchLocalMean,
                                float* __restrict__ localVarianceMap, size_t pitchLocalVariance,
                                float* __restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                const float* __restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int cellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);

    // Bounds check against local map dimensions
    if (xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
        return;

    const float* sumPtr = (const float*)((const char*)sumMap + yIndex * pitchSum) + xIndex;
    const int* countPtr = (const int*)((const char*)countMap + yIndex * pitchCount) + xIndex;
    const float* sumSqPtr = (const float*)((const char*)sumOfSquaresMap + yIndex * pitchSumSq) + xIndex;
    const float* motionVarPtr = (const float*)((const char*)motionVarianceSumMap + yIndex * pitchMotionVar) + xIndex;

    float count = *countPtr;
    float mean = 0.0f;
    float variance = 0.0f;
    float motionVariance = 0.0f;

    // Only compute if one or more points landed in this cell
    if (count > 0)
    {
        float sum = *sumPtr;
        mean = sum / count;
        motionVariance = *motionVarPtr / count;

        // If the count is only 1, there is no variance, there was only one pixel in the given cell
        if (count > 1)
        {
            // Welford's Algorithm
            float sumSq = *sumSqPtr;
            // Unbiased sample variance (one-pass), a.k.a. Bessel's Correction:
            // (Σx² - (Σx)² / N) / (N - 1)
            variance = (sumSq - (sum * sum) / static_cast<float>(count)) / (static_cast<float>(count) - 1.0f);
            variance = fmaxf(0.0f, variance); // Clamp to zero to avoid negatives from float error
        }
    }

    // Write final results to the output maps
    float* meanHeight = (float*)((char*)localMeanMap + xIndex * pitchLocalMean) + yIndex;
    float* var = (float*)((char*)localVarianceMap + xIndex * pitchLocalVariance) + yIndex;
    float* motionVar = (float*)((char*)localMotionVarianceMap + xIndex * pitchLocalMotionVariance) + yIndex;

    *meanHeight = mean;
    *var = variance;
    *motionVar = motionVariance;
}

extern "C"
__global__ void translateHeightMapKernel(float *oldHeightMapMean, size_t pitchOldHeightMapMean,
                                         float *oldHeightMapVariance, size_t pitchOldHeightMapVariance,
                                         float *oldPreviousGlobalForICP, size_t pitchOldPreviousGlobalForICP,
                                         float *previousGlobalForICP, size_t pitchPreviousGlobalForICP,
                                         float *newMeanMap, size_t pitchNewMean,
                                         float *newVarianceMap, size_t pitchNewVariance,
                                         int shiftX, int shiftY, float *params, float defaultValue)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    if (x >= globalCellsPerAxis || y >= globalCellsPerAxis)
        return;

    int srcX = x + shiftX;
    int srcY = y + shiftY;

    if (srcX >= 0 && srcX < globalCellsPerAxis && srcY >= 0 && srcY < globalCellsPerAxis)
    {
        float* oldMeanRow = (float*)((char*)oldHeightMapMean + srcX * pitchOldHeightMapMean);
        float* oldVarianceRow = (float *)((char*)oldHeightMapVariance + srcX * pitchOldHeightMapVariance);
        float* oldICPRow = (float *)((char*)oldPreviousGlobalForICP + srcX * pitchOldPreviousGlobalForICP);

        float* newMeanRow = (float*)((char*)newMeanMap + x * pitchNewMean);
        float* newVarianceRow = (float*)((char*)newVarianceMap + x * pitchNewVariance);
        float* newPreviousRow = (float*)((char*)previousGlobalForICP + x * pitchPreviousGlobalForICP);

        newMeanRow[y] = oldMeanRow[srcY];
        newPreviousRow[y] = oldICPRow[srcY];

        // Add variance due to the translation
        float increasedVariance = oldVarianceRow[srcY] + params[ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED];
        // We use min here to prevent over flow of the max value of a unsigned short
        newVarianceRow[y] = increasedVariance;
    }
    else
    {
        float* newMeanRow = (float*)((char*)newMeanMap + x * pitchNewMean);
        float* newVarianceRow = (float*)((char*)newVarianceMap + x * pitchNewVariance);
        float* newPreviousRow = (float*)((char*)previousGlobalForICP + x * pitchPreviousGlobalForICP);

        newMeanRow[y] = defaultValue;
        newVarianceRow[y] = defaultValue;
        newPreviousRow[y] = defaultValue;
    }
}

extern "C"
__global__ void heightMapRegistrationKernel(const float *__restrict__ localMeanMap, size_t pitchLocalMean,
                                            const float *__restrict__ localVarianceMap, size_t pitchLocalVariance,
                                            const float *__restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                            float *__restrict__ previousGlobalMap, size_t pitchPreviousGlobal,
                                            float *__restrict__ globalMeanMap, size_t pitchGlobalMean,
                                            float *__restrict__ globalVarianceMap, size_t pitchGlobalVariance,
                                            const float globalMapCenterX,
											const float globalMapCenterY,
											const float correctedDriftZ,
                                            const float *__restrict__ groundToWorldTranslation,
                                            const float *__restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for local indices
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    // Doing (y, x) allows for coalesced memory access when going to global memory
    int2 localCell = make_int2(yIndex, xIndex);
    float *localMean = (float *)((char *)localMeanMap + localCell.x * pitchLocalMean) + localCell.y;
    // Global memory access is asynchronous and takes a long time, tell the bus to go grab some memory
    float localMeanF = *localMean;

    // While the global memory is being fetched, convert the local cell into the global cell so we can register the data
    float2 localCoordinate = indices_to_coordinate(localCell, make_float2(0.0f, 0.0f), params[CELL_SIZE], params[LOCAL_CENTER_INDEX]);
    float3 pointInLocalFrame = make_float3(localCoordinate.x, localCoordinate.y, 0.0f);
    float3 pointInGlobalFrame = transformPoint3D(pointInLocalFrame, groundToWorldTranslation);

    // Snap to cell grid to match how the local map was built
    float snappedGlobalX = floorf(pointInGlobalFrame.x / params[CELL_SIZE]) * params[CELL_SIZE];
    float snappedGlobalY = floorf(pointInGlobalFrame.y / params[CELL_SIZE]) * params[CELL_SIZE];

    int2 globalCell = coordinate_to_indices(make_float2(snappedGlobalX, snappedGlobalY), make_float2(globalMapCenterX, globalMapCenterY), params[CELL_SIZE], params[GLOBAL_CENTER_INDEX]);

    if (globalCell.x < 0 || globalCell.x >= globalCellsPerAxis || globalCell.y < 0 || globalCell.y >= globalCellsPerAxis)
        return;

    // The work done so far by the kernel is very small, so the local mean global memory access hasn't finished yet.
    // Setup the correct address for the data we want to use later
    float *localVariance = (float *)((char *)localVarianceMap + localCell.x * pitchLocalVariance) + localCell.y;
    float *localMotionVariance = (float *)((char *)localMotionVarianceMap + localCell.x * pitchLocalMotionVariance) + localCell.y;
    float *globalMean = (float *)((char *)globalMeanMap + globalCell.x * pitchGlobalMean) + globalCell.y;
    float *globalVariance = (float *)((char *)globalVarianceMap + globalCell.x * pitchGlobalVariance) + globalCell.y;
    float *previousGlobal = (float *)((char *)previousGlobalMap + globalCell.x * pitchPreviousGlobal) + globalCell.y;

    // After trying to do as much work as possible in parallel to the global memory access. We ran out of stuff to do.
    // Check the result of global memory for invalid data, and return if not valid
    if (localMeanF == 0)
        return;

    // Access the rest of the global memory needed for the filtering, there is no other work to do
    float localVarianceF = *localVariance;
    float globalMeanF = *globalMean;
    float globalVarianceF = *globalVariance;

    // If we have no real data, we don't apply the kalman filter, just take the new real data
    if (globalVarianceF <= 0.0f)
    {
        *globalMean = localMeanF;
        *globalVariance = localVarianceF;
        *previousGlobal = localMeanF;
    }
    else
    {
        // Too many global memory access's at once seems to overwhelm the memory bus, do the last global memory access here to space things out a little
        float localMotionVarianceF = *localMotionVariance;

        // Predict step of the kalman filter
        float predictedMean = globalMeanF + correctedDriftZ;
        float predictedVariance = globalVarianceF + params[KALMAN_FILTER_PREDICTION_NOISE];

        // Update step of the kalman filter
        float kalmanGain = predictedVariance / (predictedVariance + localVarianceF + localMotionVarianceF);
        float updatedMean = predictedMean + kalmanGain * (localMeanF - predictedMean);
        float updatedVariance = (1.0f - kalmanGain) * predictedVariance;

        *globalMean = updatedMean;
        *globalVariance = updatedVariance;
        *previousGlobal = localMeanF;
    }
}

extern "C"
__global__ void heightMapEmptyRegistrationKernel(float *localMap, size_t pitchLocal,
                                                 float *globalMap, size_t pitchGlobal,
                                                 float *groundToWorldTranslation,
                                                 float *params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Compute global map size
    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    int2 localIndex = make_int2(xIndex, yIndex);
    float2 localCoordinate = indices_to_coordinate(localIndex, make_float2(0.0f, 0.0f), params[CELL_SIZE], params[LOCAL_CENTER_INDEX]);
    float3 queryPointInLocal = make_float3(localCoordinate.x, localCoordinate.y, 0.0f);
    float3 cellInGlobal = transformPoint3D(queryPointInLocal, groundToWorldTranslation);
    int2 globalIndex = coordinate_to_indices(make_float2(cellInGlobal.x, cellInGlobal.y), make_float2(groundToWorldTranslation[3], groundToWorldTranslation[7]), params[CELL_SIZE], params[GLOBAL_CENTER_INDEX]);

    if (globalIndex.x < 0 || globalIndex.x >= globalCellsPerAxis || globalIndex.y < 0 || globalIndex.y >= globalCellsPerAxis)
        return;

    float *localHeight = (float *)((char *)localMap + localIndex.x * pitchLocal) + localIndex.y;

    // This is a way of checking if we have any read data in the spot, its very unlikely that any real data is 0.0
    if (*localHeight == 0.0f)
        return;

    float *globalHeight = (float *)((char *)globalMap + globalIndex.x * pitchGlobal) + globalIndex.y;
    *globalHeight = *localHeight;
}

extern "C"
/**
    @brief Compute Plan Offset KERNEL: This kernel is not very intuitive. For background we've got a status message
    being published that gives us an offset in Z in meters that the robot thinks its drifted by.
    Our goal is to update the height map accordingly. However, it doesn't make sense to update any values that are
    in the live view of the camera, because: a) they will just get overwritten the next image, and b) that data doesn't drift
    because its live data. So we pass in this matrix of values to skip because those values are outside the live view
    of the camera.
*/
__global__ void planOffsetKernel(float *matrixToModify, size_t pitchMatrixToModify,
                                 float *matrixValuesToSkip, size_t pitchMatrixValuesToSkip,
                                 float offsetInZ, float zeroValueForEmptySpaces, float *params)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    if (indexX >= globalCellsPerAxis || indexY >= globalCellsPerAxis)
        return;

    float *skipRow = (float *)((char *)matrixValuesToSkip + indexX * pitchMatrixValuesToSkip);
    // This skips the cells that have real data in them coming from the values to skip
    if (skipRow[indexY] != zeroValueForEmptySpaces)
        return;

    float *matrixRow = (float *)((char *)matrixToModify + indexX * pitchMatrixToModify);
    matrixRow[indexY] += offsetInZ;
}