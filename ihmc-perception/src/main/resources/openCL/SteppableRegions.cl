#define HEIGHT_MAP_CENTER_INDEX 0
#define HEIGHT_MAP_RESOLUTION 1
#define CENTER_X 2
#define CENTER_Y 3
#define MIN_DISTANCE_FROM_CLIFF_TOPS 4
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 5
#define TOTAL_YAW_DISCRETIZATIONS 6
#define FOOT_WIDTH 7
#define FOOT_LENGTH 8
#define CLIFF_START_HEIGHT_TO_AVOID 9
#define CLIFF_END_HEIGHT_TO_AVOID 10

#define VALID 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define SNAP_FAILED 3

float get_yaw_from_index(global float* params, int idx_yaw)
{
    return M_PI_F * ((float) idx_yaw) / ((float) (params[TOTAL_YAW_DISCRETIZATIONS] - 1));
}

float signed_distance_to_foot_polygon(global float* params, int2 foot_key, float foot_yaw, int2 query)
{
    int cells_per_side = 2 * params[HEIGHT_MAP_CENTER_INDEX] + 1;
    int map_idx_x = cells_per_side - query.y;
    int map_idx_y = query.x;
    // we're flipping the axes because it's image vs world
    float2 vector_to_point = params[HEIGHT_MAP_RESOLUTION] * (float2) ((float) (query.x - foot_key.x), (float) (query.y - foot_key.y));

    float2 vector_in_foot_frame = applyYawRotationToVector2D(vector_to_point, -foot_yaw);

    float x_outside = fabs(vector_in_foot_frame.x) - params[FOOT_LENGTH] / 2.0f;
    float y_outside = fabs(vector_in_foot_frame.y) - params[FOOT_WIDTH] / 2.0f;

    if (x_outside > 0.0f && y_outside > 0.0f)
    {
        return length((float2) (x_outside, y_outside));
    }
    else if (x_outside > 0.0f)
    {
        return x_outside;
    }
    else if (y_outside > 0.0f)
    {
        return y_outside;
    }
    else
    {
        return max(x_outside, y_outside);
    }
}

void kernel computeSteppability(global float* params,
                                read_only image2d_t height_map,
                                global float* idx_yaw_singular_buffer,
                                write_only image2d_t steppable_map,
                                write_only image2d_t snapped_height_map,
                                write_only image2d_t snapped_normal_x_map,
                                write_only image2d_t snapped_normal_y_map,
                                write_only image2d_t snapped_normal_z_map)
{
    // Remember, these are x and y in image coordinates, not world
    int idx_x = get_global_id(0); // column
    int idx_y = get_global_id(1); // row
    int idx_yaw = (int) idx_yaw_singular_buffer[0];

    int2 key = (int2) (idx_x, idx_y);
    float foot_height = (float) read_imagef(height_map, key).x;
    float foot_yaw = get_yaw_from_index(params, idx_yaw);

    float foot_width = params[FOOT_WIDTH];
    float foot_length = params[FOOT_LENGTH];
    float distance_from_bottom = params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];
    float distance_from_top = params[MIN_DISTANCE_FROM_CLIFF_TOPS];

    float map_resolution = params[HEIGHT_MAP_RESOLUTION];
    float max_dimension = max(params[FOOT_WIDTH], params[FOOT_LENGTH]);
    int center_index = params[HEIGHT_MAP_CENTER_INDEX];
    float2 center = (float2) (params[CENTER_X], params[CENTER_Y]);
    int cells_per_side = 2 * center_index + 1;

    int map_idx_x = cells_per_side - idx_y;
    int map_idx_y = idx_x;
    int2 map_key = (int2) (map_idx_x, map_idx_y);
    float2 foot_position = indices_to_coordinate(map_key, center, map_resolution, center_index);



    // TODO check these
    float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
    int cliff_offset_indices = (int) ceil(cliff_search_offset / map_resolution);

    // search for a cliff base that's too close
    float max_height = -INFINITY;

    for (int x_query = idx_x - cliff_offset_indices; x_query <= idx_x + cliff_offset_indices; x_query++)
    {
        if (x_query < 0 || x_query >= cells_per_side)
        {
            continue;
        }
        for (int y_query = idx_y - cliff_offset_indices; y_query <= idx_y + cliff_offset_indices; y_query++)
        {
            // y is out of bounds, so skip it
            if (y_query < 0 || y_query >= cells_per_side)
            {
                continue;
            }

            // get the x,y position and height
            int2 query_key = (int2) (x_query, y_query);
            float query_height = (float) read_imagef(height_map, query_key).x;

            // compute the relative height at this point
            float relative_height = query_height - foot_height;

            float distance_to_foot = signed_distance_to_foot_polygon(params, key, foot_yaw, query_key);

            if (relative_height > params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                float height_alpha = (relative_height - params[CLIFF_START_HEIGHT_TO_AVOID]) / (params[CLIFF_END_HEIGHT_TO_AVOID] - params[CLIFF_START_HEIGHT_TO_AVOID]);
                height_alpha = clamp(height_alpha, 0.0f, 1.0f);
                float min_distance_from_this_point = height_alpha * params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];

                if (min_distance_from_this_point > distance_to_foot)
                {
                    // we're too close to the cliff bottom!
                    write_imageui(steppable_map, key, (uint4)(CLIFF_BOTTOM,0,0,0));

                    return;
                }
            }
            else if (relative_height < -params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                if (params[MIN_DISTANCE_FROM_CLIFF_TOPS] > distance_to_foot)
                {
                    // we're too close to the cliff top!
                    write_imageui(steppable_map, key, (uint4)(CLIFF_TOP,0,0,0));

                    return;
                }
            }

            //  TODO extract epsilon
            if (distance_to_foot < 1e-3f)
            {
                max_height = max(query_height, max_height);
            }
        }
    }


    int points_inside_polygon = 0;
    float running_height_total = 0.0f;
    float min_height = max_height - 0.05f;
    float resolution = 0.02f;
    float half_length = foot_length / 2.0f;
    float half_width = foot_width / 2.0f;

    // these are values that will be used for the plane calculation
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

    for (float x_value = -half_length; x_value <= half_length; x_value += resolution)
    {
        for (float y_value = -half_width; y_value <= half_width; y_value += resolution)
        {
            float2 vector_in_foot = applyYawRotationToVector2D((float2) (x_value, y_value), foot_yaw);
            float2 point_query = vector_in_foot + foot_position;

            int map_query_x = coordinate_to_index(point_query.x, center.x, map_resolution, center_index);
            int map_query_y = coordinate_to_index(point_query.y, center.y, map_resolution, center_index);
            int image_query_x = map_query_y;
            int image_query_y = cells_per_side - map_query_x;
            int2 query_key = (int2) (image_query_x, image_query_y);
            float query_height = (float) read_imagef(height_map, query_key).x;

            if (isnan(query_height) || query_height < min_height)
               continue;

            points_inside_polygon++;
            running_height_total += query_height;

            n += 1.0f;
            x += point_query.x;
            y += point_query.y;
            z += query_height;
            xx += point_query.x * point_query.x;
            xy += point_query.x * point_query.y;
            xz += point_query.x * query_height;
            yy += point_query.y * point_query.y;
            yz += point_query.y * query_height;
            zz += query_height * query_height;
        }
    }

    int lengths = (int) floor(foot_length / resolution);
    int widths = (int) floor(foot_width / resolution);
    int max_points = lengths * widths;

    // TODO extract area fraction
    float min_area_fraction = 0.75f;
    int min_points = (int) (min_area_fraction * max_points);
    if (points_inside_polygon > min_points)
    {
        float snap_height = running_height_total / points_inside_polygon;

        float covariance_matrix[9] = {xx, xy, x, xy, yy, y, x, y, n};
        float z_variance_vector[3] = {-xz, -yz, -z};
        float coefficients[3] = {0, 0, 0};
        solveForPlaneCoefficients(covariance_matrix, z_variance_vector, coefficients);

        float x_solution = x / n;
        float y_solution = y / n;
        float z_solution = -coefficients[0] * x_solution - coefficients[1] * y_solution - coefficients[2];

        float3 normal = (float3) (coefficients[0], coefficients[1], 1.0);
        normal = normalize(normal);

        // we can step here!
        write_imageui(steppable_map, key, (uint4)(VALID,0,0,0));
        write_imagef(snapped_height_map, key, (float4)(snap_height, 0.0f, 0.0f, 0.0f));
        write_imagef(snapped_normal_x_map, key, (float4)(normal.x, 0.0f, 0.0f, 0.0f));
        write_imagef(snapped_normal_x_map, key, (float4)(normal.y, 0.0f, 0.0f, 0.0f));
        write_imagef(snapped_normal_x_map, key, (float4)(normal.z, 0.0f, 0.0f, 0.0f));
    }
    else
    {
        write_imageui(steppable_map, key, (uint4)(SNAP_FAILED,0,0,0));
        write_imagef(snapped_height_map, key, (float4)(NAN, 0.0f, 0.0f, 0.0f));
        write_imagef(snapped_normal_x_map, key, (float4)(NAN, 0.0f, 0.0f, 0.0f));
        write_imagef(snapped_normal_x_map, key, (float4)(NAN, 0.0f, 0.0f, 0.0f));
        write_imagef(snapped_normal_x_map, key, (float4)(NAN, 0.0f, 0.0f, 0.0f));
    }
}

void kernel computeSteppabilityConnections(global float* params,
                                           read_only image2d_t steppable_map,
                                           write_only image2d_t steppable_connections)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int cells_per_side = 2 * params[HEIGHT_MAP_CENTER_INDEX] + 1;

    int2 key = (int2) (idx_x, idx_y);

    uint boundaryConnectionsEncodedAsOnes = (uint)(0);

    int counter = 0;
    if (read_imageui(steppable_map, key).x == VALID)
    {
        for (int x_offset = -1; x_offset <= 1; x_offset++)
        {
            for (int y_offset = -1; y_offset <= 1; y_offset++)
            {
                if (x_offset == 0 && y_offset == 0)
                    continue;

                int x_query = idx_x + x_offset;
                int y_query = idx_y + y_offset;

                // out of bounds, so skip it
                if (x_query < 0 || x_query >= cells_per_side || y_query < 0 || y_query >= cells_per_side)
                {
                    boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                }
                else
                {
                    int2 query_key = (int2) (x_query, y_query);
                    if (read_imageui(steppable_map, query_key).x == VALID)
                        boundaryConnectionsEncodedAsOnes = (1 << counter) | boundaryConnectionsEncodedAsOnes;
                    else
                        boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                }

                counter++;
            }
        }
    }

    write_imageui(steppable_connections, key, (uint4)(boundaryConnectionsEncodedAsOnes, 0, 0, 0));
}