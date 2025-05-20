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
#define ROBOT_COLLISION_RADIUS 10
#define HALF_LOCAL_WIDTH_IM_METERS 11
#define HEIGHT_FILTER_ALPHA 12
#define LOCAL_CELLS_PER_AXIS 13
#define GLOBAL_CELLS_PER_AXIS 14
#define HEIGHT_SCALING_FACTOR 15
#define MIN_HEIGHT_REGISTRATION 16
#define MAX_HEIGHT_REGISTRATION 17
#define MIN_HEIGHT_DIFFERENCE 18
#define MAX_HEIGHT_DIFFERENCE 19
#define SEARCH_WINDOW_HEIGHT 20
#define SEARCH_WINDOW_WIDTH 21
#define MIN_CLAMP_HEIGHT 22
#define MAX_CLAMP_HEIGHT 23
#define HEIGHT_OFFSET 24
#define STEPPING_COSINE_THRESHOLD 25
#define STEPPING_CONTACT_THRESHOLD 26
#define CONTACT_WINDOW_SIZE 27
#define SPATIAL_ALPHA 28
#define SEARCH_SKIP_SIZE 29
#define VERTICAL_SEARCH_SIZE 30
#define VERTICAL_SEARCH_RESOLUTION 31
#define FAST_SEARCH_SIZE 32
#define GROUND_HEIGHT 33

#define VERTICAL_FOV 1.5707963267948966f
#define HORIZONTAL_FOV 6.2831853f

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

__device__ float get_spatial_average(int xIndex, int yIndex, unsigned short *globalMap, size_t pitchGlobal, float *params)
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
                unsigned short *heightValue = (unsigned short *)((char *)globalMap + nxIndex * pitchGlobal) + nyIndex;
                heightSum += *heightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
                count++;
            }
        }
    }
    float heightAverage = heightSum / (float)count;
    return heightAverage;
}

__device__ float get_spatial_stddev(int xIndex, int yIndex, float average, unsigned short *globalMap, size_t pitchGlobal, float *params)
{
    float totalDeviation = 0.0f;
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
                unsigned short *heightValue = (unsigned short *)((char *)globalMap + nxIndex * pitchGlobal) + nyIndex;
                float height = *heightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
                totalDeviation += (height - average) * (height - average);
                count++;
            }
        }
    }
    float heightStddev = sqrt(totalDeviation / (float)count);
    return heightStddev;
}

__device__ float get_spatial_filtered_height(int xIndex, int yIndex, float height, unsigned short *globalMap, size_t pitchGlobal, float *params)
{
    float averageHeightZ = get_spatial_average(xIndex, yIndex, globalMap, pitchGlobal, params);
    float heightStddev = get_spatial_stddev(xIndex, yIndex, averageHeightZ, globalMap, pitchGlobal, params);
    float finalHeight = height;

    if (fabs(finalHeight - averageHeightZ) < 0.5f * heightStddev)
    {
        // finalHeight = averageHeightZ * params[SPATIAL_ALPHA] + finalHeight * (1.0f - params[SPATIAL_ALPHA]);
    }
    else
    {
        finalHeight = averageHeightZ * params[SPATIAL_ALPHA] * 0.0001f + finalHeight * (1.0f - params[SPATIAL_ALPHA] * 0.0001f);
    }

    return finalHeight;
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
// Iterate over a search window in the depth image to find points within the cell bounds.
// Back-project these points to the 3D space and transform them back to the Z-Up frame.
// Compute the average height for points within the grid cell while filtering outliers.
extern "C"
__global__ void heightMapUpdateKernel(unsigned short *depthImage, size_t pitchDepth,
                                      unsigned short *localMap, size_t pitchLocal,
                                      unsigned short *globalMap, size_t pitchGlobal,
                                      float *params, float *sensorToZUpFrameTf, float *zUpToSensorFrameTf,
                                      float *zUpCameraToWorldAlignedGround, float resetOffset, int bounds)
{
    // Thread indices
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Grid dimensions (from params)
    int depthWidth = static_cast<int>(params[DEPTH_INPUT_WIDTH]);
    int depthHeight = static_cast<int>(params[DEPTH_INPUT_HEIGHT]);

    // Bounds check
    if (xIndex >= bounds || yIndex >= bounds)
        return;

    // Initialize variables
    float currentAverageHeight = 0.0f;
    float averageHeightZ = 0.0f;
    int count = 0;
    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, params[GROUND_HEIGHT]);

    int2 globalIndex = getGlobalIndexFromLocalIndex(make_int2(xIndex, yIndex), zUpCameraToWorldAlignedGround, params);

    if (globalIndex.x >= 0 && globalIndex.x < static_cast<int>(params[GLOBAL_CELLS_PER_AXIS])
        && globalIndex.y >= 0 && globalIndex.y < static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]))
    {
        unsigned short *globalHeight = (unsigned short *)((char *)globalMap + globalIndex.x * pitchGlobal) + globalIndex.y;

        if (*globalHeight != resetOffset)
        {
             float unScaledHeight = static_cast<float>(*globalHeight) / params[HEIGHT_SCALING_FACTOR];
             float heightInMeters = unScaledHeight - params[HEIGHT_OFFSET];

             cellCenterInZUp.z = heightInMeters;

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
                // Read depth value using pitched memory
                unsigned short *inRow = (unsigned short *)((char *)depthImage + (pitchIdx * pitchDepth));
                unsigned short depthValue = *(inRow + yawIdx);

                // Convert depth value to meters (if necessary)
                float depth = static_cast<float>(depthValue) / 1000.0f; // Scaling depth to meters

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
                        currentAverageHeight = averageHeightZ / static_cast<float>(count);
                        if (fabsf(queryPointInZUp.z - currentAverageHeight) > 0.1f)
                            continue; // Skip if the height deviates significantly
                    }

                    count++;
                    averageHeightZ += queryPointInZUp.z;
                }
            }
        }
    }

    // Finalize average height
    if (count > 0)
    {
        averageHeightZ /= static_cast<float>(count); // Compute average height
    }
    else
    {
        averageHeightZ = -params[HEIGHT_OFFSET]; // Set to the negative height offset if no valid points
    }

    // Clamp height to the specified range
    averageHeightZ = fminf(fmaxf(averageHeightZ, params[MIN_CLAMP_HEIGHT]), params[MAX_CLAMP_HEIGHT]);

    // Apply height offset
    averageHeightZ += params[HEIGHT_OFFSET];

    // Scale to the appropriate range
    float heightValue = (averageHeightZ * params[HEIGHT_SCALING_FACTOR]);

    unsigned short *outRow = (unsigned short *)((char *)localMap + (xIndex * pitchLocal));
    *(outRow + yIndex) = (unsigned short)(heightValue);
}

extern "C"
__global__ void translateHeightMapKernel(unsigned short* oldMap, size_t pitchOld,
                                         unsigned short* newMap, size_t pitchNew,
                                         float currentCenterX, float currentCenterY,
                                         float previousCenterX, float previousCenterY,
                                         int mapSize, int defaultValue,
                                         float cellSizeInMeters)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;  // column
    int y = blockIdx.y * blockDim.y + threadIdx.y;  // row

    if (x >= mapSize || y >= mapSize)
        return;

    float dx = currentCenterX - previousCenterX;
    float dy = currentCenterY - previousCenterY;

    int shiftX = round(dx / cellSizeInMeters);
    int shiftY = round(dy / cellSizeInMeters);

    int srcX = x + shiftX;
    int srcY = y + shiftY;

    if (srcX >= 0 && srcX < mapSize && srcY >= 0 && srcY < mapSize)
    {
        unsigned short* oldRow = (unsigned short*)((char*)oldMap + srcX * pitchOld);
        unsigned short* newRow = (unsigned short*)((char*)newMap + x * pitchNew);
        newRow[y] = oldRow[srcY];
    }
    else
    {
        unsigned short* newRow = (unsigned short*)((char*)newMap + x * pitchNew);
        newRow[y] = defaultValue;
    }
}

extern "C"
__global__ void heightMapRegistrationKernel(unsigned short *localMap, size_t pitchLocal,
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

    // This performs an alpha filter on the incoming data
    float alpha = params[HEIGHT_FILTER_ALPHA];
    float localHeightF = static_cast<float>(*localHeight);
    float globalHeightF = static_cast<float>(*globalHeight);
    float filtered = alpha * localHeightF + (1.0f - alpha) * globalHeightF;

    // If we have no real data, we don't apply an alpha filter, just take the new real data
    if (globalHeightF == resetOffset)
    {
        *globalHeight = *localHeight;
    }
    else
    {
        // Add 0.5 to round to the nearest unsigned short, any decimal gets removed. e.g. 3.6 -> 4.1 = 4
        *globalHeight = static_cast<unsigned short>(filtered + 0.5f);
    }

}

extern "C"
__global__ void planOffsetKernel(unsigned short * matrixToModify, size_t pitchMatrixToModify,
                                 unsigned short * matrixValuesToSkip, size_t pitchMatrixValuesToSkip,
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