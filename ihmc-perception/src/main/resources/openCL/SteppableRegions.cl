#define HEIGHT_MAP_CENTER_INDEX 0
#define HEIGHT_MAP_RESOLUTION 1
#define MIN_DISTANCE_FROM_CLIFF_TOPS 2
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 3
#define TOTAL_YAW_DISCRETIZATIONS 4
#define FOOT_WIDTH 5
#define FOOT_LENGTH 6
#define CLIFF_START_HEIGHT_TO_AVOID 7
#define CLIFF_END_HEIGHT_TO_AVOID 8

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
                                write_only image2d_t steppable_map)
{
    // Remember, these are x and y in image coordinates, not world
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);
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

    float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
    int cliff_offset_indices = (int) ceil(cliff_search_offset / params[HEIGHT_MAP_RESOLUTION]);

    // search for a cliff base that's too close
    for (int x_query = idx_x - cliff_offset_indices; x_query <= idx_x + cliff_offset_indices; x_query++)
    {
        // x is out of bounds, so skip it
        if (x_query < 0 || x_query >= cells_per_side)
            continue;

        for (int y_query = idx_y - cliff_offset_indices; y_query <= idx_y + cliff_offset_indices; y_query++)
        {
            // y is out of bounds, so skip it
            if (y_query < 0 || y_query >= cells_per_side)
                continue;

            // get the x,y position and height
            int2 query_key = (int2) (x_query, y_query);
            float query_height = (float) read_imagef(height_map, query_key).x;

            // compute the relative height at this point
            float relative_height = query_height - foot_height;

            if (relative_height > params[CLIFF_START_HEIGHT_TO_AVOID])
            {

                float distance_to_avoid_by_alpha = (relative_height - params[CLIFF_START_HEIGHT_TO_AVOID]) / (params[CLIFF_END_HEIGHT_TO_AVOID] - params[CLIFF_START_HEIGHT_TO_AVOID]);
                float min_distance_from_this_point = distance_to_avoid_by_alpha * params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];

                float distance_to_foot = signed_distance_to_foot_polygon(params, key, foot_yaw, query_key);
                if (min_distance_from_this_point > distance_to_foot)
                {
                   printf("Cliff height %f bottom to foot %f\n", relative_height, distance_to_foot);

                    // we're too close to the cliff bottom!
                    write_imageui(steppable_map, key, (uint4)(0,0,0,0));

                    return;
                }
            }
            else if (relative_height < -params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                float distance_to_foot = signed_distance_to_foot_polygon(params, key, foot_yaw, query_key);

                if (params[MIN_DISTANCE_FROM_CLIFF_TOPS] > distance_to_foot)
                {
                    printf("Cliff top height %f to foot %f\n", relative_height, distance_to_foot);

                    // we're too close to the cliff top!
                    write_imageui(steppable_map, key, (uint4)(0,0,0,0));

                    return;
                }
            }
        }
    }

    // we can step here!
    write_imageui(steppable_map, key, (uint4)(1,0,0,0));
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

    if (read_imageui(steppable_map, key).x == 1)
    {
        for (int i = 0; i < 4; i++)
        {
            int x_query = idx_x;
            int y_query = idx_y;
            if (i == 0)
                x_query++;
            else if (i == 1)
                y_query++;
            else if (i == 2)
                x_query -= 1;
            else
                y_query -= 1;

            // out of bounds, so skip it
            if (x_query < 0 || x_query >= cells_per_side || y_query < 0 || y_query >= cells_per_side)
                continue;

            boundaryConnectionsEncodedAsOnes = (1 << i) | boundaryConnectionsEncodedAsOnes;
        }
    }

    write_imageui(steppable_connections, key, (uint4)(boundaryConnectionsEncodedAsOnes, 0, 0, 0));
}