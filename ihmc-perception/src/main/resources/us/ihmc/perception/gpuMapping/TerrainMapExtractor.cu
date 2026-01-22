#include "HeightMapUtils.cuh"
#include "MathUtils.cuh"

extern "C"

#define CELL_SIZE_IN_METERS 0
#define HEIGHT_MAP_WIDTH_IN_METERS 1
#define NORMAL_SEARCH_RADIUS 2
#define CLIFF_SEARCH_RADIUS 3
#define CLIFF_HEIGHT_THRESHOLD 4
#define CLIFF_HEIGHT_TOLERANCE 5
#define MIN_SUPPORT_AREA_FRACTION 6
#define MIN_SNAP_HEIGHT_THRESHOLD 7
#define SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE 8
#define STEPPING_COSINE_THRESHOLD 9
#define SQUARED_ERROR_THRESHOLD 10

// Reason for 0 traversability, in order it's checked
#define VALID 0
#define SNAP_FAILED 1
#define NOT_ENOUGH_AREA 2
#define SQUARED_ERROR 3
#define TOO_STEEP 4
#define CLIFF_TOP 5

/*
   This kernel is designed to compute the average snap height for every cell in the window. This can be done by either snapping a rectangular foot down if
   there's a known yaw, or, more efficiently, a circle on the ground, where you don't need to know the yaw. It also computes the local normal at that cell.
   Additionally, it performs some validity checks about the snap, specifically checking the minimum area, roughness, and terrain incline.
   The results of that check is returned in the steppable map image. When performing the snap, points that are too far below the highest point are ignored. This
   enables a better "sharp" edge around corners, to avoid rounding by averaging. It's also how the support area is calculated. In the future, the support area
   should be the area of the convex hull, not just the area of the cells, since that will allow "bridging" gaps.
*/
extern "C"
__global__ void computeTerrainData(float *heightMap, size_t pitchHeightMap,
                                   float *traversabilityMap, size_t pitchTraversability,
                                   unsigned short *traversabilityClassMap, size_t pitchTraversabilityClass,
                                   unsigned short *snapNormalXMap, size_t pitchSnapNormalX,
                                   unsigned short *snapNormalYMap, size_t pitchSnapNormalY,
                                   unsigned short *snapNormalZMap, size_t pitchSnapNormalZ,
                                   float *params, int terrainMapXY)
{
    int x_index = blockIdx.x * blockDim.x + threadIdx.x;
    int y_index = blockIdx.y * blockDim.y + threadIdx.y;

    if (x_index >= terrainMapXY || y_index >= terrainMapXY)
        return;

    float map_resolution = params[CELL_SIZE_IN_METERS];

    int terrain_map_center_index = compute_center_index(params[HEIGHT_MAP_WIDTH_IN_METERS], map_resolution);
    
    int cells_per_axis = 2 * terrain_map_center_index + 1;
    int cells_per_axis_for_checking = cells_per_axis - 1;
    int2 terrain_map_index = make_int2(x_index, y_index);

    // all logic here is relative, so absolute center is not important
    float2 terrain_map_center = make_float2(0.0f, 0.0f);

    float2 foot_position = indices_to_coordinate(terrain_map_index, terrain_map_center, map_resolution, terrain_map_center_index);
    float normal_search_radius = params[NORMAL_SEARCH_RADIUS];
    float normal_search_radius_squared = normal_search_radius * normal_search_radius;
    int index_offset = static_cast<int>(ceilf(normal_search_radius / map_resolution));

    float max_height_in_radius = -100.0f;
    int normal_search_min_x = max(terrain_map_index.x - index_offset, 0);
    int normal_search_max_x = min(terrain_map_index.x + index_offset + 1, cells_per_axis_for_checking);
    int normal_search_min_y = max(terrain_map_index.y - index_offset, 0);
    int normal_search_max_y = min(terrain_map_index.y + index_offset + 1, cells_per_axis_for_checking);

    // Get the maximum height of any cell when snapping the foot down. This gives us our highest point on the threshold. Assume the foot is a circle.
    for (int x_query = normal_search_min_x; x_query < normal_search_max_x; ++x_query)
    {
        for (int y_query = normal_search_min_y; y_query < normal_search_max_y; ++y_query)
        {
            float2 vector_to_point_from_foot = make_float2(static_cast<float>(x_query - terrain_map_index.x) * map_resolution,
                                                           static_cast<float>(y_query - terrain_map_index.y) * map_resolution);

            // If the magnitude of the vector is greater than the search radius, then the point is outside the foot.
            if (dot2D(vector_to_point_from_foot, vector_to_point_from_foot) > normal_search_radius_squared)
                continue;

            int2 query_key = make_int2(x_query, y_query);

            float *query_height_float = (float *)((char *)heightMap + query_key.y * pitchHeightMap) + query_key.x;
            max_height_in_radius = max(*query_height_float, max_height_in_radius);
        }
    }

    // Setup values to perform a least squares fit of the foot to the height map, but omitting any points that are too far below the foot.
    CovarianceData covarianceData = {};

    int max_points_possible_under_support = 0;

    for (int x_query = normal_search_min_x; x_query <= normal_search_max_x; x_query++)
    {
        for (int y_query = normal_search_min_y; y_query <= normal_search_max_y; y_query++)
        {
            // Calculate offset and check distance
            float2 offset = make_float2((float)(x_query - terrain_map_index.x) * map_resolution, (float)(y_query - terrain_map_index.y) * map_resolution);
            float offset_distance_squared = dot2D(offset, offset);

            if (offset_distance_squared > normal_search_radius_squared)
                continue;

            float offset_distance = sqrt(offset_distance_squared);
            float2 point_query = make_float2(offset.x + foot_position.x, offset.y + foot_position.y);

            float2 localXY = make_float2(point_query.x, point_query.y);
            int2 query_key  = coordinate_to_indices(localXY, terrain_map_center, map_resolution, terrain_map_center_index);

            if (query_key.x < 0 || query_key.x > cells_per_axis_for_checking || query_key.y < 0 || query_key.y > cells_per_axis_for_checking)
                continue;

            // We want to put this after the bounds check. That way, if it's outside the FOV, we don't count it against the minimum area.
            max_points_possible_under_support++;

            float *heightValue = (float *) ((char *)heightMap + query_key.y * pitchHeightMap) + query_key.x;
            float query_height = *heightValue;

            if (isnan(query_height))
                continue;

            // 0.0 in center, 1.0 at edge
            float alpha_edge = clamp(offset_distance / normal_search_radius, 0.0f, 1.0f);
            float snap_height_threshold = interpolate(params[MIN_SNAP_HEIGHT_THRESHOLD], params[SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE], alpha_edge);
            float min_height_under_foot_to_consider = max_height_in_radius - snap_height_threshold;

            if (query_height >= min_height_under_foot_to_consider)
            {
                // Using query_height yields very high squared errors for large heights due to numerical errors in the matrix inversion when plane fitting
                // Since only the relative z values are important, we subtract relative to the max height
                float z_relative = query_height - max_height_in_radius;
                covarianceData.registerPoint(point_query.x, point_query.y, z_relative);
            }
        }
    }

    // Traversability scores: support area, squared error, and incline.
    // If any are 0.0, traversability_result contains the first detected cause
    float area_traversability = 1.0f;
    float squared_error_traversability = 1.0f;
    float incline_traversability = 1.0f;
    float cliff_traversability = 1.0f;
    int traversability_result = VALID;

    // Fail if insufficient data is in the search radius
    if (covarianceData.numberOfPoints < 3)
    {
       traversability_result = SNAP_FAILED;
    }

    // Support area percentage check
    if (traversability_result == VALID)
    {
        float min_area_percentage = params[MIN_SUPPORT_AREA_FRACTION];
        float area_percentage = covarianceData.numberOfPoints / max_points_possible_under_support;
        area_traversability = clamp((area_percentage - min_area_percentage) / (1.0f - min_area_percentage), 0.0f, 1.0f);

        if (area_percentage < min_area_percentage)
        {
            traversability_result = NOT_ENOUGH_AREA;
        }
    }

    // Perform best fit plane
    float3 normal = make_float3(0.0f, 0.0f, 1.0f);
    // Output coefficients (A, B, C)
    float coefficients[3] = {0.0f, 0.0f, 0.0f};

    // Part of doing the least squares using doubles
    // double coefficients[3] = {0.0, 0.0, 0.0};

    if (traversability_result == VALID)
    {
        // Option 1 - fit plane with Cholesky
        float squared_error = solveForPlaneCoefficients3x3_Cholesky(covarianceData, coefficients);
        // Option 2 - fit plane with determinants
//         float squared_error = solveForPlaneCoefficients3x3_Determinants(covarianceData, coefficients);

        normal.x = coefficients[0];
        normal.y = coefficients[1];
        normal = normalize(normal);

        // If the normal points down, we need to flip it.
        if (normal.z < 0.0)
        {
            normal.x = -normal.x;
            normal.y = -normal.y;
            normal.z = -normal.z;
        }

        // Roughness check
        float squaredErrorThreshold = params[SQUARED_ERROR_THRESHOLD];
        squared_error_traversability = clamp((1.0f - squared_error) / squaredErrorThreshold, 0.0f, 1.0f);
        if (squared_error > squaredErrorThreshold)
        {
            traversability_result = SQUARED_ERROR;
        }

        // Incline check
        float cosineInclineThreshold = params[STEPPING_COSINE_THRESHOLD];
        float cosineIncline = normal.z;
        incline_traversability = clamp((cosineIncline - cosineInclineThreshold) / (1.0f - cosineInclineThreshold), 0.0f, 1.0f);
        if (cosineIncline < cosineInclineThreshold)
        {
            traversability_result = TOO_STEEP;
        }
    }
    
    // Check for cliffs near the best-fit plane

    float cliff_search_radius_squared = params[CLIFF_SEARCH_RADIUS] * params[CLIFF_SEARCH_RADIUS];
    float max_height_relative_to_plane = -100.0f;

    for (int x_query = normal_search_min_x; x_query <= normal_search_max_x; x_query++)
    {
        for (int y_query = normal_search_min_y; y_query <= normal_search_max_y; y_query++)
        {
            // Calculate offset and check distance
            float2 offset = make_float2((float)(x_query - terrain_map_index.x) * map_resolution, (float)(y_query - terrain_map_index.y) * map_resolution);
            float offset_distance_squared = dot2D(offset, offset);

            if (offset_distance_squared > cliff_search_radius_squared)
                continue;

            float2 point_query = make_float2(offset.x + foot_position.x, offset.y + foot_position.y);

            float2 localXY = make_float2(point_query.x, point_query.y);
            int2 query_key  = coordinate_to_indices(localXY, terrain_map_center, map_resolution, terrain_map_center_index);

            if (query_key.x < 0 || query_key.x > cells_per_axis_for_checking || query_key.y < 0 || query_key.y > cells_per_axis_for_checking)
                continue;

            float *heightValue = (float *) ((char *)heightMap + query_key.y * pitchHeightMap) + query_key.x;
            float query_height = *heightValue;

            if (isnan(query_height))
                continue;

            float plane_height = - coefficients[0] * point_query.x - coefficients[1] * point_query.y - coefficients[2] + max_height_in_radius;
            float height_relative_to_plane = query_height - plane_height;
            max_height_relative_to_plane = max(height_relative_to_plane, max_height_relative_to_plane);
        }
    }

    if (max_height_relative_to_plane > params[CLIFF_HEIGHT_THRESHOLD])
    {
        traversability_result = CLIFF_TOP;
        cliff_traversability = 0.0f;
    }
    else if (max_height_relative_to_plane > params[CLIFF_HEIGHT_TOLERANCE])
    {
        float height_past_threshold = max_height_relative_to_plane - params[CLIFF_HEIGHT_TOLERANCE];
        float max_past_threshold = params[CLIFF_HEIGHT_THRESHOLD] - params[CLIFF_HEIGHT_TOLERANCE];
        cliff_traversability = clamp(1.0f - height_past_threshold / max_past_threshold, 0.0f, 1.0f);
    }

    // Note these are switched to align with world, this is correct
    unsigned char normal_x_char = scaleAndCastToUnsignedChar(normal.y, -1.0f, 1.0f);
    unsigned char normal_y_char = scaleAndCastToUnsignedChar(normal.x, -1.0f, 1.0f);
    unsigned char normal_z_char = scaleAndCastToUnsignedChar(normal.z, 0.0f, 1.0f);

    // Pack map normal
    unsigned char *snappedNormalXMapElement = (unsigned char *)((char *)snapNormalXMap + y_index * pitchSnapNormalX) + x_index;
    *snappedNormalXMapElement = normal_x_char;

    unsigned char *snappedNormalYMapElement = (unsigned char *)((char *)snapNormalYMap + y_index * pitchSnapNormalY) + x_index;
    *snappedNormalYMapElement = normal_y_char;

    unsigned char *snappedNormalZMapElement = (unsigned char *)((char *)snapNormalZMap + y_index * pitchSnapNormalZ) + x_index;
    *snappedNormalZMapElement = normal_z_char;

    // Squared error is best indication of overall traversability
    float traversability = squared_error_traversability;

    if (traversability_result != VALID)
    {
        traversability = 0.0f;
    }

    float *traversablityMapElement = (float *)((char *)traversabilityMap + y_index * pitchTraversability) + x_index;
    *traversablityMapElement = traversability;

    unsigned char *traversabilityClassMapElement = (unsigned char *)((char *)traversabilityClassMap + y_index * pitchTraversabilityClass) + x_index;
    *traversabilityClassMapElement = static_cast<unsigned char>(traversability_result);
}
