#define HEIGHT_MAP_CENTER_INDEX 0
#define HEIGHT_MAP_RESOLUTION 1
#define MIN_DISTANCE_FROM_CLIFF_TOPS 2
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 3
#define TOTAL_YAW_DISCRETIZATIONS 4
#define FOOT_WIDTH 5
#define FOOT_LENGTH 6
#define CLIFF_START_HEIGHT_TO_AVOID 7
#define CLIFF_END_HEIGHT_TO_AVOID 8

#define VALID 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define SNAP_FAILED 3

float get_yaw_from_index(global float* params, int idx_yaw)
{
    return M_PI_2_F * ((float) (idx_yaw / params[TOTAL_YAW_DISCRETIZATIONS]));
}

float2 rotate_vector(float2 vector, float yaw)
{
    float cH = cos(yaw);
    float sH = sin(yaw);
    float dxLocal = cH * vector.x - sH * vector.y;
    float dyLocal = sH * vector.x + cH * vector.y;

    return (float2) (dxLocal, dyLocal);
}

float signed_distance_to_foot_polygon(global float* params, int2 foot_key, float foot_yaw, int2 query)
{
    float2 vector_to_point = params[HEIGHT_MAP_RESOLUTION] * (float2) ((float) (query.x - foot_key.x), (float) (query.y - foot_key.y));
    float2 vector_in_foot_frame = rotate_vector(vector_to_point, -foot_yaw);
    float x_outside = fabs(vector_in_foot_frame.x) - params[FOOT_WIDTH];
    float y_outside = fabs(vector_in_foot_frame.y) - params[FOOT_LENGTH];

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
                                write_only image2d_t snapped_height_map)
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

    float max_dimension = max(params[FOOT_WIDTH], params[FOOT_LENGTH]);
    int cells_per_side = 2 * params[HEIGHT_MAP_CENTER_INDEX] + 1;

    // TODO check these
    float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
    int cliff_offset_indices = (int) ceil(cliff_search_offset / params[HEIGHT_MAP_RESOLUTION]);

    // search for a cliff base that's too close
    float max_height = -INFINITY;
    int min_x_query = INFINITY;
    int max_x_query = -INFINITY;
    int min_y_query = INFINITY;
    int max_y_query = -INFINITY;
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
            if (distance_to_foot < 1e-5f)
            {
                min_x_query = min(x_query, min_x_query);
                max_x_query = max(x_query, max_x_query);
                min_y_query = min(y_query, min_y_query);
                max_y_query = max(y_query, max_y_query);
                max_height = max(query_height, max_height);
            }
        }
    }


    int points_inside_polygon = 0;
    float running_height_total = 0.0f;
    float min_height = max_height - 0.05f; // TODO extract
    for (int x_query = min_x_query; x_query <= max_x_query; x_query++)
    {
        for (int y_query = min_y_query; y_query <= max_y_query; y_query++)
        {
            // get the x,y position and height
            int2 query_key = (int2) (x_query, y_query);
            float query_height = (float) read_imagef(height_map, query_key).x;

            if (query_height < min_height)
                continue;

            float distance_to_foot = signed_distance_to_foot_polygon(params, key, foot_yaw, query_key);

            //  TODO extract epsilon
            if (distance_to_foot < 1e-5f)
            {
                points_inside_polygon++;
                running_height_total += query_height;
            }
        }
    }

    // TODO extract area
    float min_area = 0.75 * (params[FOOT_LENGTH] * params[FOOT_WIDTH]);
    int min_points = (int) (min_area / (params[HEIGHT_MAP_RESOLUTION] * params[HEIGHT_MAP_RESOLUTION]));
    if (points_inside_polygon > min_points)
    {
        float snap_height = running_height_total / points_inside_polygon;

        // we can step here!
        write_imageui(steppable_map, key, (uint4)(VALID,0,0,0));
        write_imagef(snapped_height_map, key, (float4)(snap_height, 0.0f, 0.0f, 0.0f));
    }
    else
    {
        write_imageui(steppable_map, key, (uint4)(SNAP_FAILED,0,0,0));
        write_imagef(snapped_height_map, key, (float4)(NAN, 0.0f, 0.0f, 0.0f));
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

    if (read_imageui(steppable_map, key).x == VALID)
    {
        for (int x_offset = -1; x_offset <= 1; x_offset++)
        {
            for (int y_offset = -1; y_offset <= 1; y_offset++)
            {
                int x_query = idx_x + x_offset;
                int y_query = idx_y + y_offset;

                // out of bounds, so skip it
                if (x_query < 0 || x_query >= cells_per_side || y_query < 0 || y_query >= cells_per_side)
                    continue;

                boundaryConnectionsEncodedAsOnes = (1 << (3 * x_offset + y_offset)) | boundaryConnectionsEncodedAsOnes;
            }
        }
    }

    write_imageui(steppable_connections, key, (uint4)(boundaryConnectionsEncodedAsOnes, 0, 0, 0));
}