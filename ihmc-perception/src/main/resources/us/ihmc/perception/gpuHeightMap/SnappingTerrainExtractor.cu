#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"

#define HEIGHT_MAP_CENTER_X 0
#define HEIGHT_MAP_CENTER_Y 1
#define CELL_SIZE_IN_CENTIMETERS 2
#define HEIGHT_MAP_WIDTH_IM_METERS 3
#define HEIGHT_SCALING_FACTOR 4
#define HEIGHT_OFFSET 5
#define FOOT_LENGTH 6
#define FOOT_WIDTH 7
#define MIN_DISTANCE_FROM_CLIFF_TOPS 8
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 9
#define CLIFF_START_HEIGHT_TO_AVOID 10
#define CLIFF_END_HEIGHT_TO_AVOID 11
#define MIN_SUPPORT_AREA_FRACTION 12
#define MIN_SNAP_HEIGHT_THRESHOLD 13
#define SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE 14
#define INEQUALITY_ACTIVATION_SLOPE 15
#define STEPPING_COSINE_THRESHOLD 16
#define STEPPING_CONTACT_THRESHOLD 17
#define CONTACT_WINDOW_SIZE 18

#define SNAP_FAILED 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define NOT_ENOUGH_AREA 0
#define VALID 4


/*
   This kernel is designed to compute the average snap height for every cell in the window. This can be done by either snapping a rectangular foot down if
   there's a known yaw, or, more efficiently, a circle on the ground, where you don't need to know the yaw. It also computes the local normal at that cell.
   Additionally, it performs some validity checks about the snap, specifically checking the minimum area, or whether it's too close to a cliff top or bottom.
   The results of that check is returned in the steppable map image. When performing the snap, points that are too far below the highest point are ignored. This
   enables a better "sharp" edge around corners, to avoid rounding by averaging. It's also how the support area is calculated. In the future, the support area
   should be the area of the convex hull, not just the area of the cells, since that will allow "bridging" gaps.
*/
extern "C"
__global__ void computeTerrainData(unsigned short *heightMap, size_t pitchHeightMap,
                                   unsigned short *steppabilityMap, size_t pitchSteppability,
                                   unsigned short *snapHeightMap, size_t pitchSnapHeight,
                                   unsigned short *snapNormalXMap, size_t pitchSnapNormalX,
                                   unsigned short *snapNormalYMap, size_t pitchSnapNormalY,
                                   unsigned short *snapNormalZMap, size_t pitchSnapNormalZ,
                                   unsigned short *snappedAreaFractionMap, size_t pitchSnappedAreaFraction,
                                   float *params, int terrainMapXY)
{
    int x_index = blockIdx.x * blockDim.x + threadIdx.x;
    int y_index = blockIdx.y * blockDim.y + threadIdx.y;

//     if (x_index >= terrainMapXY || y_index >= terrainMapXY)
//         return;

    float foot_width = params[FOOT_WIDTH];
    float foot_length = params[FOOT_LENGTH];

    float map_resolution = params[CELL_SIZE_IN_CENTIMETERS];
    float max_dimension = fmaxf(params[FOOT_WIDTH], params[FOOT_LENGTH]);

    int terrain_map_center_index = compute_center_index(params[HEIGHT_MAP_WIDTH_IM_METERS], map_resolution);
    float2 terrain_map_center = make_float2(params[HEIGHT_MAP_CENTER_Y], params[HEIGHT_MAP_CENTER_X]);

    int cells_per_axis = 2 * terrain_map_center_index + 1;
    int cells_per_axis_for_checking = cells_per_axis - 1;
    int2 terrain_map_index = make_int2(x_index, y_index);

    float2 foot_position = indices_to_coordinate(terrain_map_index, terrain_map_center, map_resolution, terrain_map_center_index);

    float half_length = foot_length / 2.0f;
    float half_width = foot_width / 2.0f;
    float2 half_foot_size = make_float2(half_length, half_width);
    float foot_search_radius_squared = dot2D(half_foot_size, half_foot_size);
    float foot_search_radius = sqrtf(foot_search_radius_squared);
    int foot_offset_indices = static_cast<int>(ceilf(foot_search_radius / map_resolution));

    int max_height_int = -100;
    int foot_search_min_x = max(terrain_map_index.x - foot_offset_indices, 0);
    int foot_search_max_x = min(terrain_map_index.x + foot_offset_indices + 1, cells_per_axis_for_checking);
    int foot_search_min_y = max(terrain_map_index.y - foot_offset_indices, 0);
    int foot_search_max_y = min(terrain_map_index.y + foot_offset_indices + 1, cells_per_axis_for_checking);

    // Get the maximum height of any cell when snapping the foot down. This gives us our highest point on the threshold. Assume the foot is a circle.
    for (int x_query = foot_search_min_x; x_query < foot_search_max_x; ++x_query)
    {
        for (int y_query = foot_search_min_y; y_query < foot_search_max_y; ++y_query)
        {
            float2 vector_to_point_from_foot = make_float2(static_cast<float>(x_query - terrain_map_index.x) * map_resolution,
                                                           static_cast<float>(y_query - terrain_map_index.y) * map_resolution);

            // If the magnitude of the vector is greater than the search radius, then the point is outside the foot.
            if (dot2D(vector_to_point_from_foot, vector_to_point_from_foot) > foot_search_radius_squared)
                continue;

            int2 query_key = make_int2(x_query, y_query);

            unsigned short *query_height_int = (unsigned short *)((char *)heightMap + query_key.y * pitchHeightMap) + query_key.x;
            max_height_int = max(*query_height_int, max_height_int);
        }
    }

    // convert that maximum height from an int representation to an actual height in the world in meters
    float max_height_under_foot = static_cast<float>(max_height_int) / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

    // Setup values to perform a least squares fit of the foot to the height map, but omitting any points that are too far below the foot.
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

            float2 localXY = make_float2(point_query.x, point_query.y);
            int2 query_key  = coordinate_to_indices(localXY, terrain_map_center, resolution, terrain_map_center_index);

            if (query_key.x < 0 || query_key.x > cells_per_axis_for_checking || query_key.y < 0 || query_key.y > cells_per_axis_for_checking)
                continue;

            // We want to put this after the bounds check. That way, if it's outside the FOV, we don't count it against the minimum area.
            max_points_possible_under_support++;

            unsigned short *heightValue = (unsigned short *) ((char *)heightMap + query_key.y * pitchHeightMap) + query_key.x;
            float query_height = (float) *heightValue / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

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
    // If the normal points down, we need to flip it.
    if (normal.z < 0.0)
    {
        normal.x = -normal.x;
        normal.y = -normal.y;
        normal.z = -normal.z;
    }

    // TODO include this?
    // snap_height = getZOnPlane(foot_position, (float3) (x_solution, y_solution, z_solution), normal);
    int snap_height_int = (snap_height + params[HEIGHT_OFFSET]) * params[HEIGHT_SCALING_FACTOR];

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
        int cliff_start_height_to_avoid_int = (params[CLIFF_START_HEIGHT_TO_AVOID]) * params[HEIGHT_SCALING_FACTOR];
        int cliff_end_height_to_avoid_int = (params[CLIFF_END_HEIGHT_TO_AVOID]) * params[HEIGHT_SCALING_FACTOR];

        float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
        float cliff_search_offset_squared = cliff_search_offset * cliff_search_offset;
        int cliff_offset_indices = (int)ceil(cliff_search_offset / map_resolution);
        float min_distance_from_tops_squared = params[MIN_DISTANCE_FROM_CLIFF_TOPS] * params[MIN_DISTANCE_FROM_CLIFF_TOPS];

        int min_x = max(terrain_map_index.x - cliff_offset_indices,0);
        int max_x = min(terrain_map_index.x + cliff_offset_indices + 1, cells_per_axis_for_checking);
        int min_y = max(terrain_map_index.y - cliff_offset_indices, 0);
        int max_y = min(terrain_map_index.y + cliff_offset_indices + 1, cells_per_axis_for_checking);

        // search for a cliff base that's too close
        for (int x_query = min_x; x_query < max_x; x_query++)
        {
            for (int y_query = min_y; y_query < max_y; y_query++)
            {
                float2 vector_to_point_from_foot = make_float2((float)(x_query - terrain_map_index.x) * map_resolution, (float)(y_query - terrain_map_index.y) * map_resolution);
                float distance_to_point_squared = dot2D(vector_to_point_from_foot, vector_to_point_from_foot);

                // skip this cell if it's too far away from the foot // , but also skip it if it's within the foot.
                if (distance_to_point_squared > cliff_search_offset_squared)
                    continue;

                int2 query_key = make_int2(x_query, y_query);

                unsigned short *heightValue = (unsigned short *) ((char *)heightMap + query_key.y * pitchHeightMap) + query_key.x;
                int query_height_int = (int) *heightValue;

                // compute the relative height at this point, compared to the height contained in the current cell.
                int relative_height_of_query_int = query_height_int - snap_height_int;

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

    // Technically speaking, the z value of the normal doesn't need to be returned, since we know the magnitude of the vector is unitary.
    int normal_x_int = static_cast<int>(255 * (normal.y + 1.0f) / 2.0f);
    int normal_y_int = static_cast<int>(255 * (normal.x + 1.0f) / 2.0f);
    int normal_z_int = static_cast<int>(255 * (normal.z + 1.0f) / 2.0f);
    int2 storage_key = make_int2(x_index, y_index);

    unsigned char *steppabilityMapElement = (unsigned char *)((char *)steppabilityMap + storage_key.y * pitchSteppability) + storage_key.x;
    *steppabilityMapElement = static_cast<unsigned char>(snap_result);

    unsigned short *snapHeightMapElement = (unsigned short *)((char *)snapHeightMap + storage_key.y * pitchSnapHeight) + storage_key.x;
    *snapHeightMapElement = static_cast<unsigned short>(snap_height_int);

    unsigned char *snappedNormalXMapElement = (unsigned char *)((char *)snapNormalXMap + storage_key.y * pitchSnapNormalX) + storage_key.x;
    *snappedNormalXMapElement = static_cast<unsigned char>(normal_x_int);

    unsigned char *snappedNormalYMapElement = (unsigned char *)((char *)snapNormalYMap + storage_key.y * pitchSnapNormalY) + storage_key.x;
    *snappedNormalYMapElement = static_cast<unsigned char>(normal_y_int);

    unsigned char *snappedNormalZMapElement = (unsigned char *)((char *)snapNormalZMap + storage_key.y * pitchSnapNormalZ) + storage_key.x;
    *snappedNormalZMapElement = static_cast<unsigned char>(normal_z_int);

    unsigned char *areaFractionElement = (unsigned char *)((char *)snappedAreaFractionMap + storage_key.y * pitchSnappedAreaFraction) + storage_key.x;
    *areaFractionElement = static_cast<unsigned char>(area_fraction);
}

extern "C"
__global__ void computeSteppabilityConnections(unsigned short *steppableMap, size_t pitchSteppableMap,
                                               unsigned short *steppableConnectionsMap, size_t pitchSteppableConnectionsMap,
                                               float* params)
{
    int x_index = blockIdx.x * blockDim.x + threadIdx.x;
    int y_index = blockIdx.y * blockDim.y + threadIdx.y;

    int cells_per_side = 2 * compute_center_index(params[HEIGHT_MAP_WIDTH_IM_METERS], params[CELL_SIZE_IN_CENTIMETERS]) + 1;

    int2 key = make_int2(x_index, y_index);

    int boundaryConnectionsEncodedAsOnes = 0;
    int counter = 0;

    unsigned char *steppability_result = (unsigned char *) ((char *)steppableMap + x_index * pitchSteppableMap) + y_index;
    if (*steppability_result == VALID)
    {
        for (int x_offset = -1; x_offset <= 1; x_offset++)
        {
            for (int y_offset = -1; y_offset <= 1; y_offset++)
            {
                if (x_offset == 0 && y_offset == 0)
                    continue;

                int x_query = x_index + x_offset;
                int y_query = y_index + y_offset;

                // Check bounds
                if (x_query < 0 || x_query >= cells_per_side || y_query < 0 || y_query >= cells_per_side)
                {
                    boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                }
                else
                {
                    int2 query_key = make_int2(x_query, y_query);
                    unsigned char *steppableValue = (unsigned char *) ((char *)steppableMap + query_key.x * pitchSteppableMap) + query_key.y;
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

    unsigned char *steppableConnectionsElement = (unsigned char *)((char *)steppableConnectionsMap + key.x * pitchSteppableConnectionsMap) + key.y;
    *steppableConnectionsElement = static_cast<unsigned char>(boundaryConnectionsEncodedAsOnes);
}

extern "C"
__global__ void computeTerrainCost(unsigned short *heightMap, size_t pitchHeightMap,
                                   unsigned char *costMap, size_t pitchCostMap,
                                   float *params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;
    int cells_per_axis = 2 * compute_center_index(params[HEIGHT_MAP_WIDTH_IM_METERS], params[CELL_SIZE_IN_CENTIMETERS]) + 1;

    // Bounds check
    if (xIndex >= cells_per_axis || yIndex >= cells_per_axis)
        return;

    float heightScalingFactor = params[HEIGHT_SCALING_FACTOR];
    float steppingCosineThreshold = params[STEPPING_COSINE_THRESHOLD];

    // Sobel operators
    const float KxSobel[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
    const float KySobel[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

    float Kx = 0.0f;
    float Ky = 0.0f;

    // Read the 3x3 neighborhood
    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            int xi = xIndex + i;
            int yj = yIndex + j;

            // Additional bounds check
            if (xi < 0 || xi >= cells_per_axis || yj < 0 || yj >= cells_per_axis)
                continue;

            unsigned short *heightPtr = (unsigned short *)((char *)heightMap + yj * pitchHeightMap) + xi;
            float heightValue = (*heightPtr) / heightScalingFactor;

            int index = (i + 1) * 3 + (j + 1);
            Kx += heightValue * KxSobel[index];
            Ky += heightValue * KySobel[index];
        }
    }

    // Surface normal
    float3 surfaceNormal;
    surfaceNormal.x = -Kx;
    surfaceNormal.y = -Ky;
    surfaceNormal.z = 1.0f;

    // Normalize surface normal
    float norm = sqrtf(surfaceNormal.x * surfaceNormal.x +
                       surfaceNormal.y * surfaceNormal.y +
                       surfaceNormal.z * surfaceNormal.z);

    if (norm > 0.0f)
    {
        surfaceNormal.x /= norm;
        surfaceNormal.y /= norm;
        surfaceNormal.z /= norm;
    }

    // Dot product with vertical (z-axis)
    float dotProduct = fabsf(surfaceNormal.z);

    // Map dotProduct to [0, 255]
    unsigned int cost = (unsigned int)(dotProduct * 255.0f);

    if (dotProduct < steppingCosineThreshold)
        cost = 0;

    unsigned char *costPtr = (unsigned char *)((char *)costMap + yIndex * pitchCostMap) + xIndex;
    *costPtr = (unsigned char)cost;
}

extern "C"
__global__ void computeContactMap(unsigned char *terrainCost, size_t pitchTerrainCost,
                                  unsigned char *contactMap, size_t pitchContactMap,
                                  float *params)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;
    int cells_per_axis = 2 * compute_center_index(params[HEIGHT_MAP_WIDTH_IM_METERS], params[CELL_SIZE_IN_CENTIMETERS]) + 1;

    if (xIndex >= cells_per_axis || yIndex >= cells_per_axis)
        return;

    int windowSize = (int)params[CONTACT_WINDOW_SIZE];
    float steppingContactThreshold = params[STEPPING_CONTACT_THRESHOLD];

    unsigned int closestDistance = 1000000;

    for (int i = -windowSize; i < windowSize; i++)
    {
        for (int j = -windowSize; j < windowSize; j++)
        {
            int x_query = xIndex + i;
            int y_query = yIndex + j;

            if (x_query >= 0 && x_query < cells_per_axis && y_query >= 0 && y_query < cells_per_axis)
            {
                unsigned char *terrainCostPtr = (unsigned char *)((char *)terrainCost + y_query * pitchTerrainCost) + x_query;
                unsigned int steppability = *terrainCostPtr;

                if (steppability <= steppingContactThreshold)
                {
                    // Euclidean distance
                    float distance = sqrtf((float)(i * i + j * j));

                    if (distance < closestDistance && distance > 3.0f)
                    {
                        closestDistance = (unsigned int)distance;
                    }
                    else if (distance < 3.0f)
                    {
                        closestDistance = 0;
                    }
                }
            }
        }
    }

    unsigned char *contactMapPtr = (unsigned char *)((char *)contactMap + yIndex * pitchContactMap) + xIndex;
    *contactMapPtr = (unsigned char)closestDistance;
}