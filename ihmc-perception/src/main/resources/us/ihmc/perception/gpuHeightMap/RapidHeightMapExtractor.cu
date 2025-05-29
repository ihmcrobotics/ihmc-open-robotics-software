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
#define HALF_LOCAL_WIDTH_IM_METERS 10
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

__device__ int2 spherical_projection(float3 cellCenter, float *params)
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

__device__ int2 perspective_projection(float3 point, float *params)
{
    float x = point.x / point.z * params[DEPTH_FX] + params[DEPTH_CX];
    float y = point.y / point.z * params[DEPTH_FY] + params[DEPTH_CY];
    return make_int2(static_cast<int>(x), static_cast<int>(y));
}

__device__ float3 back_project_spherical(int yaw_index, int pitch_index, float depth, float *params)
{
    int yawCountsFromCenter = -yaw_index - (params[DEPTH_INPUT_WIDTH] / 2);
    int pitchCountsFromCenter = -(pitch_index - (params[DEPTH_INPUT_HEIGHT] / 2));

    float yaw = yawCountsFromCenter / (float)params[DEPTH_INPUT_WIDTH] * HORIZONTAL_FOV;
    float pitch = pitchCountsFromCenter / (float)params[DEPTH_INPUT_HEIGHT] * VERTICAL_FOV;

    float r = depth * cos(pitch);

    float px = r * cos(yaw);
    float py = r * sin(yaw);
    float pz = depth * sin(pitch);

    return make_float3(px, py, pz);
}

__device__ float3 back_project_perspective(int2 pos, float Z, float *params)
{
    float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
    float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
    float3 point = make_float3(Z, -X, -Y);
    return point;
}

__device__ float get_spatial_average(int xIndex, int yIndex, unsigned short *globalHeightMap, size_t pitchGlobal, float *params)
{
    // perform a smoothing over neighboring cells
    float heightSum = 0.0f;
    int count = 0;
    int globalCellsPerAxis = (int)params[GLOBAL_CELLS_PER_AXIS];
    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            int nxIndex = xIndex + i;
            int nyIndex = yIndex + j;

            if (nxIndex >= 0 && nxIndex < globalCellsPerAxis && nyIndex >= 0 && nyIndex < globalCellsPerAxis)
            {
                unsigned short *heightValue = (unsigned short *)((char *)globalHeightMap + nxIndex * pitchGlobal) + nyIndex;
                heightSum += *heightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
                count++;
            }
        }
    }
    float heightAverage = heightSum / (float)count;
    return heightAverage;
}

__device__ int2 getGlobalIndexFromLocalIndex(int2 localIndex, float *zUpCameraToWorldAlignedGround, float* params)
{
    // The Z value doesn't matter since we are staying in Z-Up frames
    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, 0.0f);


    // Compute grid cell center in Z-Up frame
    float2 xyCoords = indices_to_coordinate(localIndex,
                                            make_float2(params[HALF_LOCAL_WIDTH_IM_METERS], 0.0f),
                                            params[CELL_SIZE],
                                            params[LOCAL_CENTER_INDEX]);

    cellCenterInZUp.x = xyCoords.x;
    cellCenterInZUp.y = xyCoords.y;

    // Transform cell center from previous Z-up to current Z-up
    float3 cellCenterInGroundNoRotation = transformPoint3D32_2(
        cellCenterInZUp,
        make_float3(zUpCameraToWorldAlignedGround[0], zUpCameraToWorldAlignedGround[1], zUpCameraToWorldAlignedGround[2]),
        make_float3(zUpCameraToWorldAlignedGround[4], zUpCameraToWorldAlignedGround[5], zUpCameraToWorldAlignedGround[6]),
        make_float3(zUpCameraToWorldAlignedGround[8], zUpCameraToWorldAlignedGround[9], zUpCameraToWorldAlignedGround[10]),
        make_float3(zUpCameraToWorldAlignedGround[3], zUpCameraToWorldAlignedGround[7], zUpCameraToWorldAlignedGround[11]));


    int2 newCellIndex = coordinate_to_indices(
        make_float2(cellCenterInGroundNoRotation.x, cellCenterInGroundNoRotation.y),
        make_float2(0.0f, 0.0f),
        params[CELL_SIZE],
        params[GLOBAL_CENTER_INDEX]);

    int2 globalIndex = make_int2(0, 0);
    globalIndex.x = newCellIndex.x;
    globalIndex.y = newCellIndex.y;

    return globalIndex;
}

// Compute grid cell center coordinates (cellCenterInZUp) in the Z-Up frame based on thread indices.
// Transform the grid cell to the sensor frame using the transformation matrix (zUpToSensorFrameTf).
// Perform projection (spherical or perspective) to map the grid cell to image indices.
// Iterate over a search window in the depth image to find points within the cell.
// Back-project these points to the 3D space and transform them back to the Z-Up frame.
// Compute the average height for points within the grid cell while filtering outliers.
extern "C"
__global__ void heightMapUpdateKernel(unsigned short *depthImage, size_t pitchDepth,
                                      float *previousGlobalHeightMap, size_t pitchGlobal,
                                      float *localMeanMap, size_t pitchLocalMean,
                                      float *localVarianceMap, size_t pitchLocalVariance,
                                      float *localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                      unsigned short *localSampleCountMap, size_t pitchLocalSampleCount,
                                      float *params, float *sensorToZUpFrameTf, float *zUpToSensorFrameTf,
                                      float *zUpCameraToWorldAlignedGround,
                                      float linearMotionMagnitude, float angularMotionMagnitude,
                                      float resetOffset)
{
    // Thread indices
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Grid dimensions (from params)
    int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);

    // Bounds check
    if (xIndex >= params[LOCAL_CELLS_PER_AXIS] || yIndex >= params[LOCAL_CELLS_PER_AXIS])
        return;

    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, params[GROUND_HEIGHT]);

    int2 globalIndex = getGlobalIndexFromLocalIndex(make_int2(xIndex, yIndex), zUpCameraToWorldAlignedGround, params);

    if (globalIndex.x >= 0 && globalIndex.x < static_cast<int>(params[GLOBAL_CELLS_PER_AXIS])
        && globalIndex.y >= 0 && globalIndex.y < static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]))
    {
        float *globalHeight = (float *)((char *)previousGlobalHeightMap + globalIndex.x * pitchGlobal) + globalIndex.y;

        if (*globalHeight != resetOffset)
        {
             cellCenterInZUp.z = *globalHeight;
        }
    }

    // Compute grid cell center in Z-Up frame
    float2 xyCoords = indices_to_coordinate(make_int2(xIndex, yIndex),
                                            make_float2(0.0f, 0.0f),
                                            params[CELL_SIZE],
                                            params[LOCAL_CENTER_INDEX]);

    cellCenterInZUp.x = xyCoords.x + params[HALF_LOCAL_WIDTH_IM_METERS];
    cellCenterInZUp.y = xyCoords.y;

    float halfCellWidth = params[CELL_SIZE] / 2.0f;
    float minX = cellCenterInZUp.x - halfCellWidth;
    float maxX = cellCenterInZUp.x + halfCellWidth;
    float minY = cellCenterInZUp.y - halfCellWidth;
    float maxY = cellCenterInZUp.y + halfCellWidth;

    int skip = static_cast<int>(params[SEARCH_SKIP_SIZE]);

    // Transform cell center from Z-Up to Sensor frame
    float3 cellCenterInSensor = transformPoint3D32_2(
        cellCenterInZUp,
        make_float3(zUpToSensorFrameTf[0], zUpToSensorFrameTf[1], zUpToSensorFrameTf[2]),
        make_float3(zUpToSensorFrameTf[4], zUpToSensorFrameTf[5], zUpToSensorFrameTf[6]),
        make_float3(zUpToSensorFrameTf[8], zUpToSensorFrameTf[9], zUpToSensorFrameTf[10]),
        make_float3(zUpToSensorFrameTf[3], zUpToSensorFrameTf[7], zUpToSensorFrameTf[11]));

    // Perform projection (spherical or perspective)
    int2 projectedPoint;
    if (params[MODE] == 0)
    { // Spherical Projection
        projectedPoint = spherical_projection(cellCenterInSensor, params);
    }
    else if (params[MODE] == 1)
    { // Perspective Projection
        float3 cellCenterInSensorZfwd = make_float3(-cellCenterInSensor.y, -cellCenterInSensor.z, cellCenterInSensor.x);
        if (cellCenterInSensorZfwd.z < 0)
            return;
        projectedPoint = perspective_projection(cellCenterInSensorZfwd, params);
    }

    // Distance from camera origin
    float distance = length(cellCenterInSensor);

    int count = 0;
    float meanZ = 0.0f;
    float m2 = 0.0f;
    float motionVarianceF = 0.0f;

    // Search within the window in the depth image
    for (int pitchOffset = -static_cast<int>(params[SEARCH_WINDOW_HEIGHT] / 2);
         pitchOffset < static_cast<int>(params[SEARCH_WINDOW_HEIGHT] / 2 + 1); pitchOffset += skip)
    {
        int pitchIdx = projectedPoint.y + pitchOffset;

        for (int yawOffset = -static_cast<int>(params[SEARCH_WINDOW_WIDTH] / 2);
             yawOffset < static_cast<int>(params[SEARCH_WINDOW_WIDTH] / 2) + 1; yawOffset += skip)
        {
            int yawIdx = projectedPoint.x + yawOffset;

            // Bounds check
            if (yawIdx >= 0 && yawIdx < depthWidth && pitchIdx >= 0 && pitchIdx < depthHeight)
            {
                unsigned short *inRow = (unsigned short *)((char *)depthImage + (pitchIdx * pitchDepth));
                unsigned short depthValue = *(inRow + yawIdx);

                float depth = static_cast<float>(depthValue) / 1000.0f; // Scaling depth to meters

                // This is roughly the minimum depth of the realsense with a little buffer
                // We reject this value cause it doesn't make any sense
                if (depth < 0.5f)
                    continue;

                // Back-project depth to 3D point
                float3 queryPointInSensor;
                queryPointInSensor = back_project_perspective(make_int2(yawIdx, pitchIdx), depth, params);

                // Transform back to Z-Up frame
                float3 queryPointInZUp = transformPoint3D32_2(
                    queryPointInSensor,
                    make_float3(sensorToZUpFrameTf[0], sensorToZUpFrameTf[1], sensorToZUpFrameTf[2]),
                    make_float3(sensorToZUpFrameTf[4], sensorToZUpFrameTf[5], sensorToZUpFrameTf[6]),
                    make_float3(sensorToZUpFrameTf[8], sensorToZUpFrameTf[9], sensorToZUpFrameTf[10]),
                    make_float3(sensorToZUpFrameTf[3], sensorToZUpFrameTf[7], sensorToZUpFrameTf[11]));

                // Check if the point is within the cell
                if (queryPointInZUp.x > minX && queryPointInZUp.x < maxX && queryPointInZUp.y > minY && queryPointInZUp.y < maxY)
                {
                    // Remove outliers and compute average height
                    if (count > 1)
                    {
                        if (fabsf(queryPointInZUp.z - meanZ) > 0.1f)
                            continue; // Skip if the height deviates significantly
                    }

                    count++;

                    // Update mean and variance using Welford's algorithm
                    float delta = queryPointInZUp.z - meanZ;
                    meanZ += delta / count;
                    float delta2 = queryPointInZUp.z - meanZ;
                    m2 += delta * delta2;
                }
            }
        }
    }

    float currentVariance = (count > 1) ? (m2 / (count - 1)) : 0.0f;

    // Motion variance
    motionVarianceF += distance * params[VARIANCE_PER_METER];
    motionVarianceF += linearMotionMagnitude * params[VARIANCE_PER_TRANSLATION_SPEED];
    motionVarianceF += angularMotionMagnitude * distance * params[VARIANCE_PER_ROTATION_SPEED];
    
    if (DEBUG && xIndex == 40 && yIndex == 40)
    {
        printf("Update Kernel -----------------------------\n");
        printf("Mean: %f\n", meanZ);
        printf("Variance: %f\n", currentVariance);
        printf("Motion Variance: %f\n", motionVarianceF);
    }

    if (count == 0)
        meanZ = 0;

    float *meanHeight = (float *)((char *)localMeanMap + xIndex * pitchLocalMean) + yIndex;
    *meanHeight = meanZ;
    float *variance = (float *)((char *)localVarianceMap + xIndex * pitchLocalVariance) + yIndex;
    *variance = currentVariance;
    float *motionVariance = (float *)((char *)localMotionVarianceMap + xIndex * pitchLocalMotionVariance) + yIndex;
    *motionVariance = motionVarianceF;
    unsigned short *numberOfSamples = (unsigned short *)((char *)localSampleCountMap + xIndex * pitchLocalSampleCount) + yIndex;
    *numberOfSamples = count;
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
__global__ void heightMapRegistrationKernel(float *localMeanMap, size_t pitchLocalMean,
                                            float *localVarianceMap, size_t pitchLocalVariance,
                                            float *localMotionVarianceMap, size_t pitchLocalMotionVariance,
                                            unsigned short *localSampleCountMap, size_t pitchLocalSampleCount,
                                            float *globalMeanMap, size_t pitchGlobalMean,
                                            float *globalVarianceMap, size_t pitchGlobalVariance,
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

    // If we have no real data, we don't apply an alpha filter, just take the new real data
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
__global__ void scalingHeightMapKernel(float *globalHeightMap, size_t pitchGlobalHeightMap,
                                       unsigned short *scaledHeightMap, size_t pitchScaledHeightMap,
                                       float *params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Compute global map size
    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= globalCellsPerAxis || yIndex >= globalCellsPerAxis)
        return;

    float *globalHeight = (float *)((char *)globalHeightMap + xIndex * pitchGlobalHeightMap) + yIndex;
    float heightClamped = fminf(fmaxf(*globalHeight, params[MIN_CLAMP_HEIGHT]), params[MAX_CLAMP_HEIGHT]);
    heightClamped += params[HEIGHT_OFFSET];
    heightClamped *= params[HEIGHT_SCALING_FACTOR];

    unsigned short *heightValue = (unsigned short *)((char *)scaledHeightMap + xIndex * pitchScaledHeightMap) + yIndex;
    *heightValue = static_cast<unsigned short>(heightClamped);
}

extern "C"
__global__ void terrainCroppingHeightMapKernel(unsigned short *globalHeightMap, size_t pitchGlobal,
                                               unsigned short *terrainMap, size_t pitchTerrain,
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

    // Access pitched memory properly
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
__global__ void planOffsetKernel(unsigned short *matrixToModify, size_t pitchMatrixToModify,
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
    if (abs( (int) skipRow[indexX] - resetOffset) >= 2)
        return;

    unsigned short *matrixRow = (unsigned short *)((char *)matrixToModify + indexY * pitchMatrixToModify);
    matrixRow[indexX] += static_cast<short>(offsetInZ * 10000.0f);
}