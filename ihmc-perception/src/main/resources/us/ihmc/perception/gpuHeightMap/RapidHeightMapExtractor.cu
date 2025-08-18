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
#define HEIGHT_SCALING_FACTOR 13
#define MIN_HEIGHT_REGISTRATION 14
#define MAX_HEIGHT_REGISTRATION 15
#define MIN_HEIGHT_DIFFERENCE 16
#define MAX_HEIGHT_DIFFERENCE 17
#define SEARCH_WINDOW_HEIGHT 18
#define SEARCH_WINDOW_WIDTH 19
#define MIN_CLAMP_HEIGHT 20
#define MAX_CLAMP_HEIGHT 21
#define HEIGHT_OFFSET 22
#define KALMAN_FILTER_PREDICTION_NOISE 23
#define ADDITIONAL_TRANSLATIONAL_VARIANCE_ADDED 24
#define VARIANCE_PER_METER 25
#define VARIANCE_PER_TRANSLATION_SPEED 26
#define VARIANCE_PER_ROTATION_SPEED 27
#define SEARCH_SKIP_SIZE 28
#define GROUND_HEIGHT 29
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
extern "C"
__global__ void heightMapUpdateKernel(const unsigned short *__restrict__ depthImage, size_t pitchDepth,
                                      float *__restrict__ previousGlobalHeightMap, size_t pitchGlobal,
                                      float *__restrict__ localMeanMap, size_t pitchLocalMean,
                                      float *__restrict__ localVarianceMap, size_t pitchLocalVariance,
                                      float *__restrict__ localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                      const float *__restrict__ params,
                                      const float *__restrict__ sensorToZUpFrameTf,
                                      const float *__restrict__ zUpToSensorFrameTf,
                                      const float *__restrict__ zUpCameraToWorldAlignedGround,
                                      const float linearMotionMagnitude,
                                      const float angularMotionMagnitude,
                                      float resetOffset)
{
    // Thread indices
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Cache params into local variables to reduce repeated global memory accesses
    const int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);
    const int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    const int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);
    const float cellSize = params[CELL_SIZE];
    const float halfLocalWidth = params[HALF_LOCAL_WIDTH_IN_METERS];
    const float groundHeight = params[GROUND_HEIGHT];
    const int searchWindowWidth = static_cast<int>(params[SEARCH_WINDOW_WIDTH]);
    const int searchWindowHeight = static_cast<int>(params[SEARCH_WINDOW_HEIGHT]);
    const int searchSkip = static_cast<int>(params[SEARCH_SKIP_SIZE]);
    const int mode = static_cast<int>(params[MODE]);
    const float varPerMeter = params[VARIANCE_PER_METER];
    const float varPerTranslationSpeed = params[VARIANCE_PER_TRANSLATION_SPEED];
    const float varPerRotationSpeed = params[VARIANCE_PER_ROTATION_SPEED];

    // Bounds check
    if (xIndex >= localCellsPerAxis || yIndex >= localCellsPerAxis)
        return;

    // Initialize cell center in Z-Up
    float cellX = 0.0f;
    float cellY = 0.0f;
    float cellZ = groundHeight;

    // Compute global index
    int2 globalIndex = getGlobalIndexFromLocalIndex(make_int2(xIndex, yIndex), zUpCameraToWorldAlignedGround, params);

    if (globalIndex.x >= 0 && globalIndex.x < globalCellsPerAxis &&
        globalIndex.y >= 0 && globalIndex.y < globalCellsPerAxis)
    {
        float *globalHeight = (float *)((char *)previousGlobalHeightMap + globalIndex.x * pitchGlobal) + globalIndex.y;
        if (*globalHeight != resetOffset)
        {
            cellZ = *globalHeight;
        }
    }

    // Compute grid cell center in local coordinates
    float2 xyCoords = indices_to_coordinate(make_int2(xIndex, yIndex),
                                            make_float2(0.0f, 0.0f),
                                            cellSize,
                                            params[LOCAL_CENTER_INDEX]);

    cellX = xyCoords.x + halfLocalWidth;
    cellY = xyCoords.y;

    float halfCellWidth = cellSize * 0.5f;
    float minX = cellX - halfCellWidth;
    float maxX = cellX + halfCellWidth;
    float minY = cellY - halfCellWidth;
    float maxY = cellY + halfCellWidth;

    // Transform cell center from Z-Up to sensor frame
    float3 cellCenterInSensor = transformPoint3D(make_float3(cellX, cellY, cellZ), zUpToSensorFrameTf);

    // Project cell to image
    int2 projectedPoint;
    if (mode == 0)
        projectedPoint = spherical_projection(cellCenterInSensor, params);
    else
    {
        float xFwd = -cellCenterInSensor.y;
        float yFwd = -cellCenterInSensor.z;
        float zFwd = cellCenterInSensor.x;
        if (zFwd < 0.0f) return;
        projectedPoint = perspective_projection(make_float3(xFwd, yFwd, zFwd), params);
    }

    // Distance from camera
    float distance = sqrtf(cellCenterInSensor.x * cellCenterInSensor.x +
                           cellCenterInSensor.y * cellCenterInSensor.y +
                           cellCenterInSensor.z * cellCenterInSensor.z);

    // Welford accumulators
    int count = 0;
    float meanZ = 0.0f;
    float m2 = 0.0f;
    float motionVarianceF = distance * varPerMeter + linearMotionMagnitude * varPerTranslationSpeed +
                            angularMotionMagnitude * distance * varPerRotationSpeed;

    int searchWindowHeightHalf = (int)(searchWindowHeight * 0.5f);
    int searchWindowWidthHalf = (int)(searchWindowWidth * 0.5f);

    // Search depth image
    for (int pitchOffset = -searchWindowHeightHalf; pitchOffset <= searchWindowHeightHalf; pitchOffset += searchSkip)
    {
        int pitchIdx = projectedPoint.y + pitchOffset;

        //  Exit the loop early if we can, optimize for performance
        if (pitchIdx < 0 || pitchIdx >= depthHeight)
            continue;

        // This is created outside the inner loop because its cheaper to only create once per loop
        unsigned short *rowPtr = (unsigned short *)((char *)depthImage + pitchIdx * pitchDepth);

        for (int yawOffset = -searchWindowWidthHalf; yawOffset <= searchWindowWidthHalf; yawOffset += searchSkip)
        {
            int yawIdx = projectedPoint.x + yawOffset;

            // Again, exit the loop early if we can
            if (yawIdx < 0 || yawIdx >= depthWidth)
                continue;

            float depth = rowPtr[yawIdx] * 0.001f; // scale to meters
            if (depth < 0.5f)
                continue;

            float3 queryPointInSensor = back_project_perspective(make_int2(yawIdx, pitchIdx), depth, params);
            float3 queryPointInZUp = transformPoint3D(queryPointInSensor, sensorToZUpFrameTf);

            if (queryPointInZUp.x > minX && queryPointInZUp.x < maxX &&
                queryPointInZUp.y > minY && queryPointInZUp.y < maxY)
            {
                count++;
                float delta = queryPointInZUp.z - meanZ;
                meanZ += delta / count;
                float delta2 = queryPointInZUp.z - meanZ;
                m2 += delta * delta2;
            }
        }
    }

    float currentVariance = (count > 1) ? (m2 / (count - 1)) : 0.0f;
    if (count == 0)
        meanZ = 0.0f;

    // Write results
    float *meanHeight = (float *)((char *)localMeanMap + xIndex * pitchLocalMean) + yIndex;
    float *variance = (float *)((char *)localVarianceMap + xIndex * pitchLocalVariance) + yIndex;
    float *motionVariance = (float *)((char *)localMotionVarianceMap + xIndex * pitchLocalMotionVariance) + yIndex;

    *meanHeight = meanZ;
    *variance = currentVariance;
    *motionVariance = motionVarianceF;
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
__global__ void scalingHeightMapKernel(const float *__restrict__ globalHeightMap, size_t pitchGlobalHeightMap,
                                       unsigned short *__restrict__ scaledHeightMap, size_t pitchScaledHeightMap,
                                       const float *__restrict__ params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Compute global map size
    const int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= globalCellsPerAxis || yIndex >= globalCellsPerAxis)
        return;

    float *globalHeight = (float *)((char *)globalHeightMap + xIndex * pitchGlobalHeightMap) + yIndex;
    // Scale the value to be visualized an an unsigned short
    float heightClamped = fminf(fmaxf(*globalHeight, params[MIN_CLAMP_HEIGHT]), params[MAX_CLAMP_HEIGHT]);
    heightClamped += params[HEIGHT_OFFSET];
    heightClamped *= params[HEIGHT_SCALING_FACTOR];

    unsigned short *heightValue = (unsigned short *)((char *)scaledHeightMap + xIndex * pitchScaledHeightMap) + yIndex;
    *heightValue = static_cast<unsigned short>(heightClamped);
}

extern "C"
__global__ void terrainCroppingHeightMapKernel(const unsigned short *__restrict__ globalHeightMap, size_t pitchGlobal,
                                               unsigned short *__restrict__ terrainMap, size_t pitchTerrain,
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
    unsigned short *globalRow = (unsigned short *)((char *)globalHeightMap + globalX * pitchGlobal);
    unsigned short *terrainRow = (unsigned short *)((char *)terrainMap + xIndex * pitchTerrain);

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