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

/**
 * @brief Height map update KERNEL: One thread per depth pixel for peak optimization.
 * Reads in the depth for each pixel in the image.
 * Back projects the points into the Z-Up frame using transforms.
 * Atomically adds the points to the maps.
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

    // Coalesced read from depth image
    const unsigned short* rowPtr = (const unsigned short*)((const char*)depthImage + yIndex * pitchDepth);
    float depth = rowPtr[xIndex] * 0.001f; // Scale to meters

    // Early exit for invalid depth
    if (depth < 0.5f)
        return;

    // Back-project and transform
    float3 queryPointInSensorFrame = back_project_perspective(make_int2(xIndex, yIndex), depth, params);
    float3 queryPointInZUpFrame = transformPoint3D(queryPointInSensorFrame, sensorToZUpFrameTf);

    // Get the correct cell index for the maps
    float2 xyCoords = make_float2(queryPointInZUpFrame.x, queryPointInZUpFrame.y);
    int2 cellIndex = coordinate_to_indices(xyCoords, make_float2(params[HALF_LOCAL_WIDTH_IN_METERS], 0.0f), cellSize, localCenterIndex);

    // Bounds check against the local map dimensions because we are scanning the entire depth image
    if (cellIndex.x < 0 || cellIndex.x >= localCellsPerAxis || cellIndex.y < 0 || cellIndex.y >= localCellsPerAxis)
        return;

    // Pointers to the target cell in each temporary map
    float* sumPtr = (float*)((char*)sumMap + cellIndex.y * pitchSum) + cellIndex.x;
    float* countPtr = (float*)((char*)countMap + cellIndex.y * pitchCount) + cellIndex.x;
    float* sumSqPtr = (float*)((char*)sumOfSquaresMap + cellIndex.y * pitchSumSq) + cellIndex.x;
    float* motionVarPtr = (float*)((char*)motionVarianceSumMap + cellIndex.y * pitchMotionVar) + cellIndex.x;

    // Atomically add this pixel's contribution
    atomicAdd(sumPtr, queryPointInZUpFrame.z);
    atomicAdd(countPtr, 1.0f);
    atomicAdd(sumSqPtr, queryPointInZUpFrame.z * queryPointInZUpFrame.z);

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
                                const float* __restrict__ countMap, size_t pitchCount,
                                const float* __restrict__ sumOfSquaresMap, size_t pitchSumSq,
                                const float* __restrict__ motionVarianceSumMap, size_t pitchMotionVar,
                                float* __restrict__ localMeanMap, size_t pitchLocalMean,
                                float* __restrict__ localVarianceMap, size_t pitchLocalVariance,
                                float* __restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                const float* __restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);

    // Bounds check against local map dimensions
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    const float* sumPtr = (const float*)((const char*)sumMap + yIndex * pitchSum) + xIndex;
    const float* countPtr = (const float*)((const char*)countMap + yIndex * pitchCount) + xIndex;
    const float* sumSqPtr = (const float*)((const char*)sumOfSquaresMap + yIndex * pitchSumSq) + xIndex;
    const float* motionVarPtr = (const float*)((const char*)motionVarianceSumMap + yIndex * pitchMotionVar) + xIndex;

    float count = *countPtr;
    float mean = 0.0f;
    float variance = 0.0f;
    float motionVariance = 0.0f;

    // Only compute if one or more points landed in this cell
    if (count > 0.5f)
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