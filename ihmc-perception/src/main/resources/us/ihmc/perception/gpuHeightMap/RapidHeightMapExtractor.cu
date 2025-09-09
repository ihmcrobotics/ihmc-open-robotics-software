#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define MODE 4
#define DEPTH_CX 5
#define DEPTH_CY 6
#define DEPTH_FX 7
#define DEPTH_FY 8
#define GLOBAL_CENTER_INDEX 9
#define HALF_LOCAL_WIDTH_IN_METERS 10
#define LOCAL_CELLS_PER_AXIS 11
#define GLOBAL_CELLS_PER_AXIS 12
#define MIN_HEIGHT_REGISTRATION 13
#define MAX_HEIGHT_REGISTRATION 14
#define MIN_HEIGHT_DIFFERENCE 15
#define MAX_HEIGHT_DIFFERENCE 16
#define SEARCH_WINDOW_HEIGHT 17
#define SEARCH_WINDOW_WIDTH 18
#define MIN_CLAMP_HEIGHT 19
#define MAX_CLAMP_HEIGHT 20
#define KALMAN_FILTER_PREDICTION_NOISE 21
#define ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED 22
#define VARIANCE_PER_METER 23
#define VARIANCE_PER_TRANSLATION_SPEED 24
#define VARIANCE_PER_ROTATION_SPEED 25
#define SEARCH_SKIP_SIZE 26
#define GROUND_HEIGHT 27

#define VERTICAL_FOV 1.5707963267948966f
#define HORIZONTAL_FOV 6.2831853f

const bool DEBUG = false;

__device__ int2 spherical_projection(float3 cellCenter, const float *params)
{
    float pitchUnit = VERTICAL_FOV / (params[DEPTH_INPUT_HEIGHT]);
    float yawUnit = HORIZONTAL_FOV / (params[DEPTH_INPUT_WIDTH]);

    int pitchOffset = params[DEPTH_INPUT_HEIGHT] / 2;
    int yawOffset = params[DEPTH_INPUT_WIDTH] / 2;

    float x = cellCenter.x;
    float y = cellCenter.y;
    float z = cellCenter.z;

    float radius = sqrt(x * x + y * y);

    float pitch = atan2f(z, radius);
    int pitchCount = (pitchOffset) - static_cast<int>(pitch / pitchUnit);

    float yaw = atan2f(-y, x);
    int yawCount = (yawOffset) + static_cast<int>(yaw / yawUnit);

    return make_int2(pitchCount, yawCount);
}

__device__ int2 perspective_projection(float3 point, const float *params)
{
    float x = point.x / point.z * params[DEPTH_FX] + params[DEPTH_CX];
    float y = point.y / point.z * params[DEPTH_FY] + params[DEPTH_CY];
    return make_int2(static_cast<int>(x), static_cast<int>(y));
}

__device__ float3 back_project_perspective(int2 pos, float Z, const float *params)
{
    float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
    float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
    float3 point = make_float3(Z, -X, -Y);
    return point;
}

__device__ int2 getGlobalIndexFromLocalIndex(int2 localIndex, const float *zUpCameraToWorldAlignedGround, const float* params)
{
    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, 0.0f);

    float2 xyCoords = indices_to_coordinate(localIndex,
                                            make_float2(params[HALF_LOCAL_WIDTH_IN_METERS], 0.0f),
                                            params[CELL_SIZE],
                                            params[LOCAL_CENTER_INDEX]);

    cellCenterInZUp.x = xyCoords.x;
    cellCenterInZUp.y = xyCoords.y;

    float3 cellCenterInGroundNoRotation = transformPoint3D(cellCenterInZUp, zUpCameraToWorldAlignedGround);

    int2 newCellIndex = coordinate_to_indices(
        make_float2(cellCenterInGroundNoRotation.x, cellCenterInGroundNoRotation.y),
        make_float2(0.0f, 0.0f),
        params[CELL_SIZE],
        params[GLOBAL_CENTER_INDEX]);

    return newCellIndex;
}

// Compute grid cell center coordinates (cellCenterInZUp) in the Z-Up frame based on thread indices.
// Transform the grid cell to the sensor frame using the transformation matrix (zUpToSensorFrameTf).
// Perform projection (spherical or perspective) to map the grid cell to image indices.
// Iterate over a search window in the depth image to find points within the cell.
// Back-project these points to the 3D space and transform them back to the Z-Up frame.
// Compute the average height for points within the grid cell while filtering outliers.

/**
 * @brief SCATTER KERNEL: One thread per depth pixel.
 * Reads depth, back-projects, transforms, and atomically adds the point's
 * contribution to the appropriate height map cell.
 */
extern "C"
__global__ void heightMapUpdateDataKernel(const unsigned short* __restrict__ depthImage, size_t pitchDepth,
                                          float* __restrict__ sumMap, size_t pitchSum,
                                          float* __restrict__ countMap, size_t pitchCount,
                                          float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                          float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                          const float* __restrict__ params,
                                          const float* __restrict__ sensorToZUpFrameTf,
                                          const float linearMotionMagnitude,
                                          const float angularMotionMagnitude)
{
    // Thread indices now correspond to PIXEL coordinates
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Cache params
    const int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    const int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);
    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const float cellSize = params[CELL_SIZE];
    const int localCenterIndex = static_cast<int>(params[LOCAL_CENTER_INDEX]);
    const float varPerMeter = params[VARIANCE_PER_METER];
    const float varPerTranslationSpeed = params[VARIANCE_PER_TRANSLATION_SPEED];
    const float varPerRotationSpeed = params[VARIANCE_PER_ROTATION_SPEED];

    // Bounds check against depth image dimensions
    if (xIndex >= depthWidth || yIndex >= depthHeight)
        return;

    // --- 1. Coalesced Read from Depth Image ---
    const unsigned short* rowPtr = (const unsigned short*)((const char*)depthImage + yIndex * pitchDepth);
    float depth = rowPtr[xIndex] * 0.001f; // scale to meters

    if (depth < 0.5f) // Early exit for invalid depth
        return;

    // --- 2. Back-project and Transform (same as before) ---
    float3 queryPointInSensor = back_project_perspective(make_int2(xIndex, yIndex), depth, params);
    float3 queryPointInZUp = transformPoint3D(queryPointInSensor, sensorToZUpFrameTf);

    // --- 3. Find Target Cell ---
    float2 xyCoords = make_float2(queryPointInZUp.x, queryPointInZUp.y);
    int2 cellIndex = coordinate_to_indices(xyCoords, make_float2(params[HALF_LOCAL_WIDTH_IN_METERS], 0.0f), cellSize, localCenterIndex);

    // Bounds check against the local map dimensions
    if (cellIndex.x < 0 || cellIndex.x >= localCellsPerAxis || cellIndex.y < 0 || cellIndex.y >= localCellsPerAxis)
        return;

    // --- 4. Atomic Updates ---
    // Pointers to the target cell in each intermediate map
    float* sumPtr = (float*)((char*)sumMap + cellIndex.y * pitchSum) + cellIndex.x;
    float* countPtr = (float*)((char*)countMap + cellIndex.y * pitchCount) + cellIndex.x;
    float* sumSqPtr = (float*)((char*)sumOfSquaresMap + cellIndex.y * pitchSumSq) + cellIndex.x;
    float* motionVarPtr = (float*)((char*)motionVarianceSumMap + cellIndex.y * pitchMotionVar) + cellIndex.x;

    // Atomically add this pixel's contribution
    atomicAdd(sumPtr, queryPointInZUp.z);
    atomicAdd(countPtr, 1.0f);
    atomicAdd(sumSqPtr, queryPointInZUp.z * queryPointInZUp.z);

    // Also calculate and add motion variance contribution
    float distance = sqrtf(queryPointInSensor.x * queryPointInSensor.x + queryPointInSensor.y * queryPointInSensor.y + queryPointInSensor.z * queryPointInSensor.z);
    float motionVarianceF = distance * varPerMeter + linearMotionMagnitude * varPerTranslationSpeed +
                            angularMotionMagnitude * distance * varPerRotationSpeed;
    atomicAdd(motionVarPtr, motionVarianceF);
}


/**
 * @brief FINALIZE KERNEL: One thread per height map cell.
 * Reads the intermediate sum/count buffers and computes the final
 * mean, variance, and motion variance.
 */
extern "C"
__global__ void computeLocalMap(const float* __restrict__ sumMap, size_t pitchSum,
                                const float* __restrict__ countMap, size_t pitchCount,
                                const float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                const float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                float* __restrict__ localMeanMap, size_t pitchLocalMean,
                                float* __restrict__ localVarianceMap, size_t pitchLocalVariance,
                                float* __restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                const float* __restrict__ params)
{
    // Thread indices now correspond to CELL coordinates
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);

    // Bounds check against local map dimensions
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    // Pointers to the source cell in each intermediate map
    const float* sumPtr = (const float*)((const char*)sumMap + yIndex * pitchSum) + xIndex;
    const float* countPtr = (const float*)((const char*)countMap + yIndex * pitchCount) + xIndex;
    const float* sumSqPtr = (const float*)((const char*)sumOfSquaresMap + yIndex * pitchSumSq) + xIndex;
    const float* motionVarPtr = (const float*)((const char*)motionVarianceSumMap + yIndex * pitchMotionVar) + xIndex;

    float count = *countPtr;
    float mean = 0.0f;
    float variance = 0.0f;
    float motionVariance = 0.0f;

    // Only compute if one or more points landed in this cell
    if (count > 0.5f) // Use float comparison
    {
        float sum = *sumPtr;
        mean = sum / count;
        motionVariance = *motionVarPtr / count;

        if (count > 1.5f)
        {
            float sumSq = *sumSqPtr;
            // Stable one-pass variance formula: (E[X^2] - (E[X])^2 * N) / (N-1)
            variance = (sumSq - (sum * sum) / count) / (count - 1.0f);
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

        float* newMeanRow = (float*)((char*)newMeanMap + x * pitchNewMean);
        float* newVarianceRow = (float*)((char*)newVarianceMap + x * pitchNewVariance);

        newMeanRow[y] = oldMeanRow[srcY];

        // Add variance due to the translation
        float increasedVariance = oldVarianceRow[srcY] + params[ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED];
        // We use min here to prevent over flow of the max value of a unsigned short
        newVarianceRow[y] = increasedVariance;
    }
    else
    {
        float* newMeanRow = (float*)((char*)newMeanMap + x * pitchNewMean);
        float* newVarianceRow = (float*)((char*)newVarianceMap + x * pitchNewVariance);

        newMeanRow[y] = defaultValue;
        newVarianceRow[y] = defaultValue;
    }
}

extern "C"
__global__ void heightMapRegistrationKernel(const float *__restrict__ localMeanMap, size_t pitchLocalMean,
                                            const float *__restrict__ localVarianceMap, size_t pitchLocalVariance,
                                            const float *__restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                            float *__restrict__ globalMeanMap, size_t pitchGlobalMean,
                                            float *__restrict__ globalVarianceMap, size_t pitchGlobalVariance,
                                            const float *__restrict__ zUpCameraToWorldAlignedGround,
                                            const float *__restrict__ params, float resetOffset)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    // Compute global map size
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    int2 localIndex = make_int2(xIndex, yIndex);

    int2 globalIndex = getGlobalIndexFromLocalIndex(localIndex, zUpCameraToWorldAlignedGround, params);

    if (globalIndex.x < 0 || globalIndex.x >= globalCellsPerAxis || globalIndex.y < 0 || globalIndex.y >= globalCellsPerAxis)
        return;

    float *localMean = (float *)((char *)localMeanMap + localIndex.x * pitchLocalMean) + localIndex.y;
    float *localVariance = (float *)((char *)localVarianceMap + localIndex.x * pitchLocalVariance) + localIndex.y;
    float *localMotionVariance = (float *)((char *)localMotionVarianceMap + localIndex.x * pitchLocalMotionVariance) + localIndex.y;

    if (*localMean == 0)
        return;

    float *globalMean = (float *)((char *)globalMeanMap + globalIndex.x * pitchGlobalMean) + globalIndex.y;
    float *globalVariance = (float *)((char *)globalVarianceMap + globalIndex.x * pitchGlobalVariance) + globalIndex.y;

    float localMeanF = static_cast<float>(*localMean);
    float localVarianceF = static_cast<float>(*localVariance);
    float localMotionVarianceF = static_cast<float>(*localMotionVariance);
    float globalMeanF = static_cast<float>(*globalMean);
    float globalVarianceF = static_cast<float>(*globalVariance);

    // If we have no real data, we don't apply the kalman filter, just take the new real data
    if (globalMeanF == resetOffset || globalVarianceF <= 0.0f)
    {
        *globalMean = *localMean;
        *globalVariance = *localVariance;
    }
    else
    {
        // Predict step of the filter
        float predictedMean = globalMeanF;
        float predictedVariance = globalVarianceF + params[KALMAN_FILTER_PREDICTION_NOISE];

        // Update step of the filter
        float kalmanGain = predictedVariance / (predictedVariance + localVarianceF + localMotionVarianceF);
        float updatedMean = predictedMean + kalmanGain * (localMeanF - predictedMean);
        float updatedVariance = (1.0f - kalmanGain) * predictedVariance;

        if (DEBUG && xIndex == 40 && yIndex == 40)
        {
            printf("Registration Kernel -----------------------------\n");
            printf("Global Mean: %f\n", globalMeanF);
            printf("Kalman Gain: %f\n", kalmanGain);
            printf("New Mean: %f\n", updatedMean);
            printf("New Variance: %f\n", updatedVariance);
        }

        *globalMean = updatedMean;
        *globalVariance = updatedVariance;
    }
}

extern "C"
__global__ void terrainCroppingHeightMapKernel(const float *__restrict__ globalHeightMap, size_t pitchGlobal,
                                               float *__restrict__ terrainMap, size_t pitchTerrain,
                                               int centerIndexTerrain, float *params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    int terrainCellsPerAxis = 2 * centerIndexTerrain + 1;
    if (xIndex >= terrainCellsPerAxis || yIndex >= terrainCellsPerAxis)
        return;

    // Compute coordinates in the global map
    int globalX = params[GLOBAL_CENTER_INDEX] - centerIndexTerrain + xIndex;
    int globalY = params[GLOBAL_CENTER_INDEX] - centerIndexTerrain + yIndex;

    // Set the terrain map index to the equivalent index in the global map
    float *globalRow = (float *)((char *)globalHeightMap + globalX * pitchGlobal);
    float *terrainRow = (float *)((char *)terrainMap + xIndex * pitchTerrain);

    terrainRow[yIndex] = globalRow[globalY];
}

extern "C"
__global__ void heightMapEmptyRegistrationKernel(unsigned short *localMap, size_t pitchLocal,
                                                 unsigned short *globalMap, size_t pitchGlobal,
                                                 float *zUpCameraToWorldAlignedGround,
                                                 float *params, float resetOffset)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Compute global map size
    int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    int2 localIndex = make_int2(xIndex, yIndex);

    int2 globalIndex = getGlobalIndexFromLocalIndex(localIndex, zUpCameraToWorldAlignedGround, params);

    if (globalIndex.x < 0 || globalIndex.x >= globalCellsPerAxis || globalIndex.y < 0 || globalIndex.y >= globalCellsPerAxis)
        return;

    unsigned short *localHeight = (unsigned short *)((char *)localMap + localIndex.x * pitchLocal) + localIndex.y;

    if (*localHeight == 0)
        return;

    unsigned short *globalHeight = (unsigned short *)((char *)globalMap + globalIndex.x * pitchGlobal) + globalIndex.y;
    *globalHeight = *localHeight;
}

extern "C"
__global__ void planOffsetKernel(float *matrixToModify, size_t pitchMatrixToModify,
                                 unsigned short *matrixValuesToSkip, size_t pitchMatrixValuesToSkip,
                                 float offsetInZ, int rowsMatrixToModify, int colsMatrixToModify,
                                 float resetOffset)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;

    if (indexX >= colsMatrixToModify || indexY >= rowsMatrixToModify)
        return;

    unsigned short *skipRow = (unsigned short *)((char *)matrixValuesToSkip + indexY * pitchMatrixValuesToSkip);
    // This is less then or equal to due to a round error that can give +- 1 offsets
    // This skips the cells that have real data in them coming from the values to skip
    if (abs((int) skipRow[indexX] - resetOffset) >= 2)
        return;

    float *matrixRow = (float *)((char *)matrixToModify + indexY * pitchMatrixToModify);
    matrixRow[indexX] += offsetInZ;
}