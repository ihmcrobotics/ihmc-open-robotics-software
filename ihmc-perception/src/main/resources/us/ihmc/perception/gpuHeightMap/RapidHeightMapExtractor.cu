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

#define VERTICAL_FOV 1.5707963267948966f
#define HORIZONTAL_FOV 6.2831853f

#define SNAP_HEIGHT_MAP_CENTER_X 0
#define SNAP_HEIGHT_MAP_CENTER_Y 1
#define SNAP_GLOBAL_CELL_SIZE 2
#define SNAP_GLOBAL_CENTER_INDEX 3
#define SNAP_CROPPED_WINDOW_CENTER_INDEX 4
#define SNAP_HEIGHT_SCALING_FACTOR 5
#define SNAP_HEIGHT_OFFSET 6
#define SNAP_FOOT_LENGTH 7
#define SNAP_FOOT_WIDTH 8
#define MIN_DISTANCE_FROM_CLIFF_TOPS 9
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 10
#define CLIFF_START_HEIGHT_TO_AVOID 11
#define CLIFF_END_HEIGHT_TO_AVOID 12
#define MIN_SUPPORT_AREA_FRACTION 13
#define MIN_SNAP_HEIGHT_THRESHOLD 14
#define SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE 15
#define INEQUALITY_ACTIVATION_SLOPE 16

#define SNAP_FAILED 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define NOT_ENOUGH_AREA 0
#define VALID 4

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

__device__ float get_spatial_average(int xIndex, int yIndex, unsigned short *globalMap, float *params)
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
                int index = nyIndex * globalCellsPerAxis + nxIndex;
                heightSum += globalMap[index] / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
                count++;
            }
        }
    }
    float heightAverage = heightSum / (float)count;
    return heightAverage;
}

__device__ float get_spatial_stddev(int xIndex, int yIndex, float average, unsigned short *globalMap, float *params)
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
                int index = nyIndex * globalCellsPerAxis + nxIndex;
                float height = globalMap[index] / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
                totalDeviation += (height - average) * (height - average);
                count++;
            }
        }
    }
    float heightStddev = sqrt(totalDeviation / (float)count);
    return heightStddev;
}

__device__ float get_spatial_filtered_height(int xIndex, int yIndex, float height, unsigned short *globalMap, float *params)
{
    float averageHeightZ = get_spatial_average(xIndex, yIndex, globalMap, params);
    float heightStddev = get_spatial_stddev(xIndex, yIndex, averageHeightZ, globalMap, params);
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

// Compute grid cell center coordinates (cellCenterInZUp) in the Z-Up frame based on thread indices.
// Transform the grid cell to the sensor frame using the transformation matrix (zUpToSensorFrameTf).
// Perform projection (spherical or perspective) to map the grid cell to image indices.
// Iterate over a search window in the depth image to find points within the cell bounds.
// Back-project these points to the 3D space and transform them back to the Z-Up frame.
// Compute the average height for points within the grid cell while filtering outliers.

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
    float3 cellCenterInZUp = make_float3(0.0f, 0.0f, 0.5f);

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

extern "C" __global__ void heightMapRegistrationKernel(unsigned short *localMap, size_t pitchLocal,
                                                       unsigned short *globalMap, size_t pitchGlobal,
                                                       float *params, float *worldToZUpFrameTf,
                                                       float *sensorToGroundTf)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Compute global map size
    int globalCellsPerAxis = static_cast<int>(params[GLOBAL_CELLS_PER_AXIS]);

    // Check bounds for global indices
    if (xIndex >= globalCellsPerAxis || yIndex >= globalCellsPerAxis)
        return;

    // Create a 3D point in world frame
    float3 cellCenterInWorld = make_float3(0, 0, 0);

    float2 tempCoord = indices_to_coordinate(
        make_int2(xIndex, yIndex),
        make_float2(0.0f, 0.0f),
        params[GLOBAL_CELL_SIZE],
        params[GLOBAL_CENTER_INDEX]);

    cellCenterInWorld.x = tempCoord.x;
    cellCenterInWorld.y = tempCoord.y;

    // Transform the point to the ZUp frame
    float3 cellCenterInZUpFrame = transformPoint3D32_2(
        cellCenterInWorld,
        make_float3(worldToZUpFrameTf[0], worldToZUpFrameTf[1], worldToZUpFrameTf[2]),
        make_float3(worldToZUpFrameTf[4], worldToZUpFrameTf[5], worldToZUpFrameTf[6]),
        make_float3(worldToZUpFrameTf[8], worldToZUpFrameTf[9], worldToZUpFrameTf[10]),
        make_float3(worldToZUpFrameTf[3], worldToZUpFrameTf[7], worldToZUpFrameTf[11]));

    // Check collision
    float2 cellCenterInZUpFrameXY = make_float2(cellCenterInZUpFrame.x, cellCenterInZUpFrame.y);

    bool isColliding = length2D(cellCenterInZUpFrameXY) < params[ROBOT_COLLISION_RADIUS];
    if (isColliding)
        return;

    // Offset ZUp frame X coordinate and calculate local indices
    cellCenterInZUpFrame.x -= params[GRID_OFFSET_X];

    int2 localCellIndex = coordinate_to_indices(
        make_float2(cellCenterInZUpFrame.x, cellCenterInZUpFrame.y),
        make_float2(0.0f, 0.0f),
        params[LOCAL_CELL_SIZE],
        params[LOCAL_CENTER_INDEX]);

    int localCellsPerAxis = static_cast<int>(params[LOCAL_CELLS_PER_AXIS]);

    // Retrieve local height and global height
    float sensorHeight = sensorToGroundTf[11] - 1.5f;

    unsigned short *heightValue = (unsigned short *)((char *)globalMap + xIndex * pitchGlobal) + yIndex;
    float previousHeight = *heightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
    float localHeight = previousHeight;

    if (localCellIndex.x >= 0 && localCellIndex.x < localCellsPerAxis &&
        localCellIndex.y >= 0 && localCellIndex.y < localCellsPerAxis)
    {

        unsigned short *newHeightValue = (unsigned short *)((char *)localMap + localCellIndex.x * pitchLocal) + localCellIndex.y;
        localHeight = *newHeightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
    }

    float finalHeight = previousHeight;

    // Filter the height
    if (!isColliding && (localHeight - sensorHeight) > params[MIN_HEIGHT_REGISTRATION] &&
        (localHeight - sensorHeight) < params[MAX_HEIGHT_REGISTRATION])
    {
        float height_diff = fabsf(localHeight - previousHeight);
        if (height_diff < params[MAX_HEIGHT_DIFFERENCE])
        {
            finalHeight = previousHeight * params[HEIGHT_FILTER_ALPHA] +
                          localHeight * (1.0f - params[HEIGHT_FILTER_ALPHA]);
        }
        else
        {
            finalHeight = localHeight;
        }
    }

    finalHeight += params[HEIGHT_OFFSET];

    // Store the final height in the global map
    unsigned short *globalMapElement = (unsigned short *)((char *)globalMap + xIndex * pitchGlobal) + yIndex;
    *globalMapElement = static_cast<unsigned short>(finalHeight * params[HEIGHT_SCALING_FACTOR]);
}

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


const float PI_F = 3.1415927f;

__device__ float get_yaw_from_index(int yaw_discretizations, int idx_yaw)
{
    return PI_F * ((float)idx_yaw) / ((yaw_discretizations - 1));
}

// This kernel is designed to compute the average snap height for every cell in the window. This can be done by either snapping a rectangular foot down if
// there's a known yaw, or, more efficiently, a circle on the ground, where you don't need to know the yaw. It also computes the local normal at that cell.
// Additionally, it performs some validity checks about the snap, specifically checking the minimum area, or whether it's too close to a cliff top or bottom.
// The results of that check is returned in the steppable map image. When performing the snap, points that are too far below the highest point are ignored. This
// enables a better "sharp" edge around corners, to avoid rounding by averaging. It's also how the support area is calculated. In the future, the support area
// should be the area of the convex hull, not just the area of the cells, since that will allow "bridging" gaps.
extern "C"
__global__ void computeSnappedValuesKernel(unsigned short *globalMap, size_t pitchGlobal,
                                           unsigned short *steppabilityMap, size_t pitchSteppability,
                                           unsigned short *snapHeightMap, size_t pitchSnapHeight,
                                           unsigned short *snapNormalXMap, size_t pitchSnapNormalX,
                                           unsigned short *snapNormalYMap, size_t pitchSnapNormalY,
                                           unsigned short *snapNormalZMap, size_t pitchSnapNormalZ,
                                           unsigned short *snappedAreaFractionMap, size_t pitchSnappedAreaFraction,
                                           float *params)
{
    int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
    int idx_y = blockIdx.y * blockDim.y + threadIdx.y;

    if (idx_x >= 201 || idx_y >= 201)
        return;

    bool should_print = false;//idx_x == 20 && idx_y == 20;

    int2 key = make_int2(idx_x, idx_y);

    float foot_width = params[SNAP_FOOT_WIDTH];
    float foot_length = params[SNAP_FOOT_LENGTH];

    float map_resolution = params[SNAP_GLOBAL_CELL_SIZE];
    float max_dimension = fmaxf(params[SNAP_FOOT_WIDTH], params[SNAP_FOOT_LENGTH]);
    int map_center_index = static_cast<int>(params[SNAP_GLOBAL_CENTER_INDEX]);
    int cropped_center_index = static_cast<int>(params[SNAP_CROPPED_WINDOW_CENTER_INDEX]);
    float2 center = make_float2(params[SNAP_HEIGHT_MAP_CENTER_Y], params[SNAP_HEIGHT_MAP_CENTER_X]);
    float2 map_center = make_float2(0.0f, 0.0f);

    int map_cells_per_side = 2 * map_center_index + 1;
    int map_cells_per_side_for_checking = map_cells_per_side - 1;

    int crop_idx_x = idx_x;
    int crop_idx_y = idx_y;
    int2 crop_key = make_int2(crop_idx_x, crop_idx_y);

    float2 foot_position = indices_to_coordinate(crop_key, center, map_resolution, cropped_center_index);

    // Convert from the world coordinate to the map index.
    int2 map_key = coordinate_to_indices(foot_position, map_center, map_resolution, map_center_index);

    float half_length = foot_length / 2.0f;
    float half_width = foot_width / 2.0f;
    float2 half_foot_size = make_float2(half_length, half_width);
    float foot_search_radius_squared = dot2D(half_foot_size, half_foot_size);
    float foot_search_radius = sqrtf(foot_search_radius_squared);
    int foot_offset_indices = static_cast<int>(ceilf(foot_search_radius / map_resolution));

    int max_height_int = -100;
    int foot_search_min_x = max(map_key.x - foot_offset_indices, 0);
    int foot_search_max_x = min(map_key.x + foot_offset_indices + 1, map_cells_per_side_for_checking);
    int foot_search_min_y = max(map_key.y - foot_offset_indices, 0);
    int foot_search_max_y = min(map_key.y + foot_offset_indices + 1, map_cells_per_side_for_checking);

    for (int x_query = foot_search_min_x; x_query < foot_search_max_x; ++x_query)
    {
        for (int y_query = foot_search_min_y; y_query < foot_search_max_y; ++y_query)
        {
            float2 vector_to_point_from_foot = make_float2(static_cast<float>(x_query - map_key.x) * map_resolution,
                                                           static_cast<float>(y_query - map_key.y) * map_resolution);

            if (dot2D(vector_to_point_from_foot, vector_to_point_from_foot) > foot_search_radius_squared)
                continue;

            int2 query_key = make_int2(x_query, y_query);

            unsigned short *query_height_int = (unsigned short *)((char *)globalMap + query_key.y * pitchGlobal) + query_key.x;
            max_height_int = max(*query_height_int, max_height_int);
        }
    }

    float max_height_under_foot = static_cast<float>(max_height_int) / params[SNAP_HEIGHT_SCALING_FACTOR] - params[SNAP_HEIGHT_OFFSET];

    // Setup values
    float n = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float xx = 0.0f;
    float xy = 0.0f;
    float xz = 0.0f;
    float yy = 0.0f;
    float yz = 0.0f;
    float zz = 0.0f;

    int max_points_possible_under_support = 0;

    int samples = 5;
    float resolution = foot_search_radius / samples;

    for (int x_value_idx = -samples; x_value_idx <= samples; x_value_idx++)
    {
        for (int y_value_idx = -samples; y_value_idx <= samples; y_value_idx++)
        {
            // Calculate offset and check distance
            float2 offset = make_float2((float)x_value_idx * resolution, (float)y_value_idx * resolution);
            float offset_distance_squared = dot2D(offset, offset);

            if (offset_distance_squared > foot_search_radius_squared)
                continue;

            float offset_distance = sqrt(offset_distance_squared);
            float2 point_query = make_float2(offset.x + foot_position.x, offset.y + foot_position.y);

            int2 query_key = coordinate_to_indices(point_query, map_center, map_resolution, map_center_index);

            if (query_key.x < 0 || query_key.x > map_cells_per_side_for_checking || query_key.y < 0 || query_key.y > map_cells_per_side_for_checking)
                continue;

            // We want to put this after the bounds check. That way, if it's outside the FOV, we don't count it against the minimum area.
            max_points_possible_under_support++;

            unsigned short *heightValue = (unsigned short *) ((char *)globalMap + query_key.y * pitchGlobal) + query_key.x;
            float query_height = (float) *heightValue / params[SNAP_HEIGHT_SCALING_FACTOR] - params[SNAP_HEIGHT_OFFSET];

            if (isnan(query_height))
                continue;

            float snap_height_threshold = params[MIN_SNAP_HEIGHT_THRESHOLD] + params[SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE] * fminf(fmaxf(offset_distance / foot_search_radius, 0.0f), 1.0f);
            float min_height_under_foot_to_consider = max_height_under_foot - snap_height_threshold;

            // This activation gain is a way of doing a soft inequality. If the query height is less than the min height, as an inequality constraint, the
            // activation value is zero, and if it's greater, the activation is 1.0. In this formulation, we're blurring around that hard inequality. If the
            // query height is less than the min height, the "error" is negative, so the tanh function returns -1.0f. If it's positive, tanh returns 1.0f.
            float tanh_slope = params[INEQUALITY_ACTIVATION_SLOPE];
            float activation = 0.5f * (1.0f + tanh(tanh_slope * (query_height - min_height_under_foot_to_consider)));

            float activation2 = activation * activation;

            n += activation;
            x += activation * point_query.x;
            y += activation * point_query.y;
            z += activation * query_height;
            xx += activation2 * point_query.x * point_query.x;
            xy += activation2 * point_query.x * point_query.y;
            xz += activation2 * point_query.x * query_height;
            yy += activation2 * point_query.y * point_query.y;
            yz += activation2 * point_query.y * query_height;
            zz += activation2 * query_height * query_height;
        }
    }

    ///////////// Solve for the plane normal, as well as the height of the foot along that plane.
    bool failed = false;
    int snap_result = VALID;

    // Fixme this arguably should never happen
    if (n < 0.0001f)
    {
        snap_result = SNAP_FAILED;
        failed = true;
        n = 1.0f;
    }

    // This is the actual height of the snapped foot
    float snap_height = z / n;

    float covariance_matrix[9] = {xx, xy, x, xy, yy, y, x, y, n};
    float z_variance_vector[3] = {-xz, -yz, -z};
    float coefficients[3] = {0.0f, 0.0f, 0.0f};
    solveForPlaneCoefficients(covariance_matrix, z_variance_vector, coefficients);

    float3 normal = make_float3(coefficients[0], coefficients[1], 1.0);
    normal = normalize(normal);

    // TODO include this?
    // snap_height = getZOnPlane(foot_position, (float3) (x_solution, y_solution, z_solution), normal);
    int snap_height_int = (snap_height + params[SNAP_HEIGHT_OFFSET]) * params[SNAP_HEIGHT_SCALING_FACTOR];

    /////////////// Make sure there's enough step area.

    float min_points_needed_for_support = (int)(params[MIN_SUPPORT_AREA_FRACTION] * max_points_possible_under_support);
    if (n < min_points_needed_for_support)
    {
        snap_result = NOT_ENOUGH_AREA;
        failed = true;
    }

    //////////// Check to make sure we're not stepping too near a cliff base or top
    if (!failed)
    {
        int cliff_start_height_to_avoid_int = (params[CLIFF_START_HEIGHT_TO_AVOID]) * params[SNAP_HEIGHT_SCALING_FACTOR];
        int cliff_end_height_to_avoid_int = (params[CLIFF_END_HEIGHT_TO_AVOID]) * params[SNAP_HEIGHT_SCALING_FACTOR];

        float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
        float cliff_search_offset_squared = cliff_search_offset * cliff_search_offset;
        int cliff_offset_indices = (int)ceil(cliff_search_offset / map_resolution);
        float min_distance_from_tops_squared = params[MIN_DISTANCE_FROM_CLIFF_TOPS] * params[MIN_DISTANCE_FROM_CLIFF_TOPS];

        int min_x = max(map_key.x - cliff_offset_indices,0);
        int max_x = min(map_key.x + cliff_offset_indices + 1, map_cells_per_side_for_checking);
        int min_y = max(map_key.y - cliff_offset_indices, 0);
        int max_y = min(map_key.y + cliff_offset_indices + 1, map_cells_per_side_for_checking);

        // search for a cliff base that's too close
        for (int x_query = min_x; x_query < max_x; x_query++)
        {
            for (int y_query = min_y; y_query < max_y; y_query++)
            {
                float2 vector_to_point_from_foot = make_float2((float)(x_query - map_key.x) * map_resolution, (float)(y_query - map_key.y) * map_resolution);
                float distance_to_point_squared = dot2D(vector_to_point_from_foot, vector_to_point_from_foot);

                // skip this cell if it's too far away from the foot // , but also skip it if it's within the foot.
                if (distance_to_point_squared > cliff_search_offset_squared)
                    continue;

                int2 query_key = make_int2(x_query, y_query);

                unsigned short *heightValue = (unsigned short *) ((char *)globalMap + query_key.y * pitchGlobal) + query_key.x;
                int query_height_int = (int) *heightValue;

                // compute the relative height at this point, compared to the height contained in the current cell.
                int relative_height_of_query_int = query_height_int - snap_height_int;

                if (should_print)
                {
                   printf("actually checking if a cliff now. relative height is %d\n", relative_height_of_query_int);
                }

                if (relative_height_of_query_int > cliff_start_height_to_avoid_int)
                {
                    float height_alpha = (relative_height_of_query_int - cliff_start_height_to_avoid_int) / (cliff_end_height_to_avoid_int - cliff_start_height_to_avoid_int);
                    height_alpha = fminf(fmaxf(height_alpha, 0.0f), 1.0f);
                    float min_distance_from_this_point_to_avoid_cliff = height_alpha * params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];

                    if (distance_to_point_squared < min_distance_from_this_point_to_avoid_cliff * min_distance_from_this_point_to_avoid_cliff)
                    {
                        // we're too close to the cliff bottom!
                        snap_result = CLIFF_BOTTOM;
                        failed = true;
                        break;
                    }
                }
                else if (relative_height_of_query_int < -cliff_start_height_to_avoid_int)
                {
                    if (distance_to_point_squared < min_distance_from_tops_squared)
                    {
                        snap_result = CLIFF_TOP;
                        failed = true;
                        break;
                    }
                }
            }

            if (failed)
                break;
        }
    }

    // Add remaining logic, preserving the structure of the OpenCL kernel and adapting to CUDA constructs.

    // Write results back to surfaces.
    int area_fraction = static_cast<int>(255 * n / max_points_possible_under_support);
    // note these are switched to align with world

    int normal_x_int = static_cast<int>(255 * (normal.y + 1.0f) / 2.0f);
    int normal_y_int = static_cast<int>(255 * (normal.x + 1.0f) / 2.0f);
    int normal_z_int = static_cast<int>(255 * (normal.z + 1.0f) / 2.0f);
    int2 storage_key = make_int2(idx_x, idx_y);

    unsigned short *steppabilityMapElement = (unsigned short *)((char *)steppabilityMap + storage_key.y * pitchSteppability) + storage_key.x;
    *steppabilityMapElement = static_cast<unsigned short>(snap_result);

    unsigned short *snapHeightMapElement = (unsigned short *)((char *)snapHeightMap + storage_key.y * pitchSnapHeight) + storage_key.x;
    *snapHeightMapElement = static_cast<unsigned short>(snap_height_int);

    unsigned short *snappedNormalXMapElement = (unsigned short *)((char *)snapNormalXMap + storage_key.y * pitchSnapNormalX) + storage_key.x;
    *snappedNormalXMapElement = static_cast<unsigned short>(normal_x_int);

    unsigned short *snappedNormalYMapElement = (unsigned short *)((char *)snapNormalYMap + storage_key.y * pitchSnapNormalY) + storage_key.x;
    *snappedNormalYMapElement = static_cast<unsigned short>(normal_y_int);

    unsigned short *snappedNormalZMapElement = (unsigned short *)((char *)snapNormalZMap + storage_key.y * pitchSnapNormalZ) + storage_key.x;
    *snappedNormalZMapElement = static_cast<unsigned short>(normal_z_int);

    unsigned short *areaFractionElement = (unsigned short *)((char *)snappedAreaFractionMap + storage_key.y * pitchSnappedAreaFraction) + storage_key.x;
    *areaFractionElement = static_cast<unsigned short>(area_fraction);
}

extern "C"
__global__ void computeSteppabilityConnectionsKernel(float* params,
                                                     unsigned short *steppableMap, size_t pitchSteppableMap,
                                                     unsigned short *steppableConnectionsMap, size_t pitchSteppableConnectionsMap)
{
    int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
    int idx_y = blockIdx.y * blockDim.y + threadIdx.y;

    int cells_per_side = 2 * static_cast<int>(params[SNAP_CROPPED_WINDOW_CENTER_INDEX]) + 1;

    int2 key = make_int2(idx_x, idx_y);

    int boundaryConnectionsEncodedAsOnes = 0;

    int counter = 0;

    unsigned short *heightValue = (unsigned short *) ((char *)steppableMap + idx_x * pitchSteppableMap) + idx_y;
    if (*heightValue == VALID)
    {
        for (int x_offset = -1; x_offset <= 1; x_offset++)
        {
            for (int y_offset = -1; y_offset <= 1; y_offset++)
            {
                if (x_offset == 0 && y_offset == 0)
                    continue;

                int x_query = idx_x + x_offset;
                int y_query = idx_y + y_offset;

                // Check bounds
                if (x_query < 0 || x_query >= cells_per_side || y_query < 0 || y_query >= cells_per_side)
                {
                    boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                }
                else
                {
                    int2 query_key = make_int2(x_query, y_query);
                    unsigned short *steppableValue = (unsigned short *) ((char *)steppableMap + query_key.x * pitchSteppableMap) + query_key.y;
                    if (*steppableValue == VALID)
                    {
                        boundaryConnectionsEncodedAsOnes = (1 << counter) | boundaryConnectionsEncodedAsOnes;
                    }
                    else
                    {
                        boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                    }
                }

                counter++;
            }
        }
    }

        unsigned short *steppableConnectionsElement = (unsigned short *)((char *)steppableConnectionsMap + key.x * pitchSteppableConnectionsMap) + key.y;
        *steppableConnectionsElement = static_cast<unsigned short>(boundaryConnectionsEncodedAsOnes);
}