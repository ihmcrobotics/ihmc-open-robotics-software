#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"
#define LOCAL_CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define HEIGHT_MAP_CENTER_X 4
#define HEIGHT_MAP_CENTER_Y 5
#define MODE 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define DEPTH_FX 9
#define DEPTH_FY 10
#define GLOBAL_CELL_SIZE 11
#define GLOBAL_CENTER_INDEX 12
#define ROBOT_COLLISION_RADIUS 13
#define GRID_OFFSET_X 14
#define HEIGHT_FILTER_ALPHA 15
#define LOCAL_CELLS_PER_AXIS 16
#define GLOBAL_CELLS_PER_AXIS 17
#define HEIGHT_SCALING_FACTOR 18
#define MIN_HEIGHT_REGISTRATION 19
#define MAX_HEIGHT_REGISTRATION 20
#define MIN_HEIGHT_DIFFERENCE 21
#define MAX_HEIGHT_DIFFERENCE 22
#define SEARCH_WINDOW_HEIGHT 23
#define SEARCH_WINDOW_WIDTH 24
#define CROPPED_WINDOW_CENTER_INDEX 25
#define MIN_CLAMP_HEIGHT 26
#define MAX_CLAMP_HEIGHT 27
#define HEIGHT_OFFSET 28
#define STEPPING_COSINE_THRESHOLD 29
#define STEPPING_CONTACT_THRESHOLD 30
#define CONTACT_WINDOW_SIZE 31
#define SPATIAL_ALPHA 32
#define SEARCH_SKIP_SIZE 33
#define VERTICAL_SEARCH_SIZE 34
#define VERTICAL_SEARCH_RESOLUTION 35
#define FAST_SEARCH_SIZE 36
#define GROUND_HEIGHT 37

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

extern "C" __global__ void preprocessImageKernel(unsigned short *in, size_t pitchIn, unsigned short *out, size_t pitchOut, int rows, int cols)
{
    // Thread indices for output image
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x; // Output dimension x (new cols)
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y; // Output dimension y (new rows)

    // Ensure within bounds of the output image (1280 rows and 720 columns)
    if (xIndex >= rows || yIndex >= cols)
    {
        return;
    }

    unsigned short *inRow = (unsigned short *)((char *)in + (yIndex * pitchIn));
    unsigned short depthValue = *(inRow + xIndex);

    int outRow = xIndex; // Flipping and rotation (yIndex becomes the new row)
    int outCol = cols - 1 - yIndex;

    unsigned short *outRowPtr = (unsigned short *)((char *)out + (outRow * pitchOut));
    *(outRowPtr + outCol) = depthValue;
}

// Compute grid cell center coordinates (cellCenterInZUp) in the Z-Up frame based on thread indices.
// Transform the grid cell to the sensor frame using the transformation matrix (zUpToSensorFrameTf).
// Perform projection (spherical or perspective) to map the grid cell to image indices.
// Iterate over a search window in the depth image to find points within the cell bounds.
// Back-project these points to the 3D space and transform them back to the Z-Up frame.
// Compute the average height for points within the grid cell while filtering outliers.
extern "C" __global__ void heightMapUpdateKernel(unsigned short *in, size_t pitchIn, unsigned short *out, size_t pitchOut, float *params, float *sensorToZUpFrameTf, float *zUpToSensorFrameTf, int bounds)
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
    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, params[GROUND_HEIGHT] + 0.5f);

    // Compute grid cell center in Z-Up frame
    float2 xyCoords = indices_to_coordinate(make_int2(xIndex, yIndex),
                                            make_float2(0.0f, 0.0f),
                                            params[LOCAL_CELL_SIZE],
                                            params[LOCAL_CENTER_INDEX]);

    cellCenterInZUp.x = xyCoords.x + params[GRID_OFFSET_X];
    cellCenterInZUp.y = xyCoords.y;

    float halfCellWidth = params[LOCAL_CELL_SIZE] / 2.0f;
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
                unsigned short *inRow = (unsigned short *)((char *)in + (pitchIdx * pitchIn));
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

    unsigned short *outRow = (unsigned short *)((char *)out + (xIndex * pitchOut));
    *(outRow + yIndex) = (unsigned short)(heightValue);
}

extern "C"
__global__ void heightMapRegistrationKernel(unsigned short *localMap, size_t pitchLocal,
                                            unsigned short *globalMap, size_t pitchGlobal,
                                            float *params)
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

    int2 globalIndex = make_int2(0, 0);
    globalIndex.x = localIndex.x - params[LOCAL_CENTER_INDEX] + params[GLOBAL_CENTER_INDEX];
    globalIndex.y = localIndex.y - params[LOCAL_CENTER_INDEX] + params[GLOBAL_CENTER_INDEX];


    if (globalIndex.x < 0 || globalIndex.x >= globalCellsPerAxis || globalIndex.y < 0 || globalIndex.y >= globalCellsPerAxis)
        return;

    unsigned short *localHeight = (unsigned short *)((char *)localMap + localIndex.x * pitchLocal) + localIndex.y;
    unsigned short *globalHeight = (unsigned short *)((char *)globalMap + globalIndex.x * pitchGlobal) + globalIndex.y;

    *globalHeight = *localHeight;
}

extern "C"
__global__ void shiftGlobalMapKernel(unsigned short* oldMap, size_t pitchOld,
                                     unsigned short* newMap, size_t pitchNew,
                                     float previousCenterX, float previousCenterY,
                                     int mapSize, float *params, float *worldToZUpFrameTf)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;  // column
    int y = blockIdx.y * blockDim.y + threadIdx.y;  // row

    if (x >= mapSize || y >= mapSize)
        return;

    float dx = params[HEIGHT_MAP_CENTER_X] - previousCenterX;
    float dy = params[HEIGHT_MAP_CENTER_Y] - previousCenterY;

    if (x == 20 && y ==20)
    {
        printf("New Center: %f, %f\n", params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]);
        printf("Old Center: %f, %f\n", previousCenterX, previousCenterY);
    }

    int shiftX = round(dx / params[GLOBAL_CELL_SIZE]);
    int shiftY = round(dy / params[GLOBAL_CELL_SIZE]);

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
        newRow[y] = 0;
    }
}

// extern "C"
// __global__ void heightMapRegistrationKernel(unsigned short *localMap, size_t pitchLocal,
//                                             unsigned short *globalMap, size_t pitchGlobal,
//                                             float *params, float *worldToZUpFrameTf,
//                                             float *sensorToGroundTf)
// {
//     int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
//     int yIndex = blockIdx.y * blockDim.y + threadIdx.y;
//
//     // Compute global map size
//     int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);
//
//     // Check bounds for global indices
//     if (xIndex >= globalCellsPerAxis || yIndex >= globalCellsPerAxis)
//         return;
//
//     // Create a 3D point in world frame
//     float3 cellCenterInWorld = make_float3(0, 0, 0);
//
//     float2 tempCoord = indices_to_coordinate(
//         make_int2(xIndex, yIndex),
//         make_float2(0.0f, 0.0f),
//         params[GLOBAL_CELL_SIZE],
//         params[GLOBAL_CENTER_INDEX]);
//
//     cellCenterInWorld.x = tempCoord.x;
//     cellCenterInWorld.y = tempCoord.y;
//
//     // Transform the point to the ZUp frame
//     float3 cellCenterInZUpFrame = transformPoint3D32_2(
//         cellCenterInWorld,
//         make_float3(worldToZUpFrameTf[0], worldToZUpFrameTf[1], worldToZUpFrameTf[2]),
//         make_float3(worldToZUpFrameTf[4], worldToZUpFrameTf[5], worldToZUpFrameTf[6]),
//         make_float3(worldToZUpFrameTf[8], worldToZUpFrameTf[9], worldToZUpFrameTf[10]),
//         make_float3(worldToZUpFrameTf[3], worldToZUpFrameTf[7], worldToZUpFrameTf[11]));
//
//     // Check collision
//     float2 cellCenterInZUpFrameXY = make_float2(cellCenterInZUpFrame.x, cellCenterInZUpFrame.y);
//
//     bool isColliding = length2D(cellCenterInZUpFrameXY) < params[ROBOT_COLLISION_RADIUS];
//     if (isColliding)
//         return;
//
//     // Offset ZUp frame X coordinate and calculate local indices
//     cellCenterInZUpFrame.x -= params[GRID_OFFSET_X];
//
//     int2 localCellIndex = coordinate_to_indices(
//         make_float2(cellCenterInZUpFrame.x, cellCenterInZUpFrame.y),
//         make_float2(0.0f, 0.0f),
//         params[LOCAL_CELL_SIZE],
//         params[LOCAL_CENTER_INDEX]);
//
//     int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);
//
//     // Retrieve local height and global height
//     float sensorHeight = sensorToGroundTf[11] - 1.5f;
//
//     unsigned short *heightValue = (unsigned short *)((char *)globalMap + xIndex * pitchGlobal) + yIndex;
//     float previousHeight = *heightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
//     float localHeight = previousHeight;
//
//     if (localCellIndex.x >= 0 && localCellIndex.x < localCellsPerAxis &&
//         localCellIndex.y >= 0 && localCellIndex.y < localCellsPerAxis)
//     {
//
//         unsigned short *newHeightValue = (unsigned short *)((char *)localMap + localCellIndex.x * pitchLocal) + localCellIndex.y;
//         localHeight = *newHeightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
//     }
//
//     float finalHeight = previousHeight;
//
//     if (!isColliding && (localHeight - sensorHeight) > params[MIN_HEIGHT_REGISTRATION] &&
//         (localHeight - sensorHeight) < params[MAX_HEIGHT_REGISTRATION])
//     {
//         finalHeight = localHeight;
//     }
//
//     finalHeight = get_spatial_filtered_height(xIndex, yIndex, finalHeight, globalMap, pitchGlobal, params);
//
//     finalHeight += params[HEIGHT_OFFSET];
//
//     // Store the final height in the global map
//     unsigned short *globalMapElement = (unsigned short *)((char *)globalMap + xIndex * pitchGlobal) + yIndex;
//     *globalMapElement = static_cast<unsigned short>(finalHeight * params[HEIGHT_SCALING_FACTOR]);
// }

extern "C" __global__ void croppingKernel(unsigned short *inputMap, size_t pitchInput,
                                          unsigned short *croppedMap, size_t pitchCropped,
                                          float *params, int croppedMapXY)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    int globalMapSizeX = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);
    int globalMapSizeY = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    if (xIndex >= croppedMapXY || yIndex >= croppedMapXY)
        return;

    int2 globalSensorIndex = coordinate_to_indices(
        make_float2(params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]),
        make_float2(0.0f, 0.0f),
        params[GLOBAL_CELL_SIZE],
        static_cast<int>(params[GLOBAL_CENTER_INDEX]));

    int globalCellIndexX = globalSensorIndex.x + xIndex - (params[CROPPED_WINDOW_CENTER_INDEX]);
    int globalCellIndexY = globalSensorIndex.y + yIndex - (params[CROPPED_WINDOW_CENTER_INDEX]);

    // Check if global cell index is within bounds
    if (globalCellIndexX >= 0 && globalCellIndexX < globalMapSizeX &&
        globalCellIndexY >= 0 && globalCellIndexY < globalMapSizeY)
    {
        unsigned short *inputRow = (unsigned short *)((char *)inputMap + globalCellIndexX * pitchInput);
        unsigned short *croppedRow = (unsigned short *)((char *)croppedMap + xIndex * pitchCropped);
        croppedRow[yIndex] = inputRow[globalCellIndexY];
    }
    else
    {
        unsigned short *croppedRow = (unsigned short *)((char *)croppedMap + xIndex * pitchCropped);
        croppedRow[yIndex] = 0; // Assign 0 for out-of-bounds cells
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