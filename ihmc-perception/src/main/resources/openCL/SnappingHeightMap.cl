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
#define FOOT_LENGTH 330
#define FOOT_WIDTH 34
#define MIN_DISTANCE_FROM_CLIFF_TOPS 35
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 36
#define CLIFF_START_HEIGHT_TO_AVOID 37
#define CLIFF_END_HEIGHT_TO_AVOID 38

#define VERTICAL_FOV M_PI_2_F
#define HORIZONTAL_FOV (2.0f * M_PI_F)

#define VALID 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define SNAP_FAILED 3

#define X_TO_PRINT 773
#define Y_TO_PRINT 843

float get_yaw_from_index(int yaw_discretizations, int idx_yaw)
{
    return M_PI_F * ((float) idx_yaw) / ((yaw_discretizations - 1));
}

float signed_distance_to_foot_polygon(int center_index, float resolution, global float* params, int2 foot_key, float foot_yaw, int2 query)
{
    int cells_per_side = 2 * center_index + 1;
    int map_idx_x = cells_per_side - query.y;
    int map_idx_y = query.x;
    // we're flipping the axes because it's image vs world
    float2 vector_to_point = resolution * (float2) ((float) (query.x - foot_key.x), (float) (query.y - foot_key.y));

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

float get_height_on_plane(float x, float y, global float *plane)
{
   float height = (plane[3] - (plane[0] * x + plane[1] * y)) / plane[2];
   return height;
}

void kernel computeSnappedValuesKernel(global float* params,
                                read_write image2d_t height_map,
                                global float* idx_yaw_singular_buffer,
                                read_write image2d_t steppable_map,
                                read_write image2d_t snapped_height_map,
                                read_write image2d_t snapped_normal_x_map,
                                read_write image2d_t snapped_normal_y_map,
                                read_write image2d_t snapped_normal_z_map)
{
    // Remember, these are x and y in image coordinates, not world
    int idx_x = get_global_id(0); // column
    int idx_y = get_global_id(1); // row
    int idx_yaw = (int) idx_yaw_singular_buffer[0]; // TODO not needed

    int2 key = (int2) (idx_x, idx_y);
    uint foot_height_int = read_imageui(height_map, key).x;
    if (foot_height_int == 32768) // FIXME this is the placeholder for no data
    {
       write_imageui(steppable_map, key, (uint4)(SNAP_FAILED,0,0,0));

       write_imageui(snapped_height_map, key, (uint4)(0, 0, 0, 0));
       write_imageui(snapped_normal_x_map, key, (uint4)(0, 0, 0, 0));
       write_imageui(snapped_normal_y_map, key, (uint4)(0, 0, 0, 0));
       write_imageui(snapped_normal_z_map, key, (uint4)(0, 0, 0, 0));

       return;
    }
    float foot_height = (float) foot_height_int / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

    bool should_print = false;//idx_x == 0 && idx_y == 0;

    if (should_print)
    {
        printf("got to foot height\n");
    }

    // TODO yaw discretizations should get set
    float foot_yaw = get_yaw_from_index(2, idx_yaw);

    float foot_width = params[FOOT_WIDTH];
    float foot_length = params[FOOT_LENGTH];
    float distance_from_bottom = params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];
    float distance_from_top = params[MIN_DISTANCE_FROM_CLIFF_TOPS];

    float map_resolution = params[GLOBAL_CELL_SIZE];
    float max_dimension = max(params[FOOT_WIDTH], params[FOOT_LENGTH]);
    int center_index = params[GLOBAL_CENTER_INDEX];
    float2 center = (float2) (params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]);
    int cells_per_side = 2 * center_index + 1;

    int map_idx_x = cells_per_side - idx_y;
    int map_idx_y = idx_x;
    int2 map_key = (int2) (map_idx_x, map_idx_y);
    float2 foot_position = indices_to_coordinate(map_key, center, map_resolution, center_index);

    if (should_print)
    {
       printf("got foot position %f, %f\n", foot_position.x, foot_position.y);
    }


    // TODO check these
    float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
    int cliff_offset_indices = (int) ceil(cliff_search_offset / map_resolution);

    // search for a cliff base that's too close
    float max_height = -INFINITY;

if (should_print)
    {
    printf("starting search for cliff\n");
    }

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
            float query_height = (float) read_imageui(height_map, query_key).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

            // compute the relative height at this point
            float relative_height = query_height - foot_height;

            float distance_to_foot = signed_distance_to_foot_polygon(center_index, map_resolution, params, key, foot_yaw, query_key);

            if (relative_height > params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                float height_alpha = (relative_height - params[CLIFF_START_HEIGHT_TO_AVOID]) / (params[CLIFF_END_HEIGHT_TO_AVOID] - params[CLIFF_START_HEIGHT_TO_AVOID]);
                height_alpha = clamp(height_alpha, 0.0f, 1.0f);
                float min_distance_from_this_point = height_alpha * params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];

                if (min_distance_from_this_point > distance_to_foot)
                {
                    // we're too close to the cliff bottom!
                    write_imageui(steppable_map, key, (uint4)(CLIFF_BOTTOM,0,0,0));

                    write_imageui(snapped_height_map, key, (uint4)(0, 0, 0, 0));
                    write_imageui(snapped_normal_x_map, key, (uint4)(0, 0, 0, 0));
                    write_imageui(snapped_normal_y_map, key, (uint4)(0, 0, 0, 0));
                    write_imageui(snapped_normal_z_map, key, (uint4)(0, 0, 0, 0));

                    return;
                }
            }
            else if (relative_height < -params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                if (params[MIN_DISTANCE_FROM_CLIFF_TOPS] > distance_to_foot)
                {
                    // we're too close to the cliff top!
                    write_imageui(steppable_map, key, (uint4)(CLIFF_TOP,0,0,0));

                    write_imageui(snapped_height_map, key, (uint4)(0, 0, 0, 0));
                    write_imageui(snapped_normal_x_map, key, (uint4)(0, 0, 0, 0));
                    write_imageui(snapped_normal_y_map, key, (uint4)(0, 0, 0, 0));
                    write_imageui(snapped_normal_z_map, key, (uint4)(0, 0, 0, 0));

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

if (should_print)
    {
    printf("finished search for cliff\n");
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

    if (should_print)
    {
  printf("starting search for area\n");
  }

    for (float x_value = -half_length; x_value <= half_length; x_value += resolution)
    {
        for (float y_value = -half_width; y_value <= half_width; y_value += resolution)
        {


            if (should_print)
            {
                printf("finding vector in foot with (%f, %f), %f\n", x_value, y_value, foot_yaw);
            }
            float2 vector_in_foot = applyYawRotationToVector2D((float2) (x_value, y_value), foot_yaw);

            if (should_print)
            {
                printf("found vector in foot (%f, %f)\n", vector_in_foot.x, vector_in_foot.y);
                printf("foot position (%f, %f)\n", foot_position.x, foot_position.y);
            }
            float2 point_query = vector_in_foot + foot_position;
            if (should_print)
            {
                printf("point query (%f, %f)\n", point_query.x, point_query.y);
            }

            int map_query_x = coordinate_to_index(point_query.x, center.x, map_resolution, center_index);
            int map_query_y = coordinate_to_index(point_query.y, center.y, map_resolution, center_index);

            if (should_print)
            {
                printf("map query %d, %d\n", map_query_x, map_query_y);
            }
            // FIXME this will cause over sampling of certain points. Instead, if it's out of bounds, we should just skip this point
            int image_query_x = clamp(map_query_y, 0, cells_per_side - 1);
            int image_query_y = clamp(cells_per_side - map_query_x, 0, cells_per_side - 1);
             if (should_print)
                        {
                            printf("image query %d, %d\n", image_query_x, image_query_y);
                        }
            int2 query_key = (int2) (image_query_x, image_query_y);
            uint query_height_int = read_imageui(height_map, query_key).x;
            if (should_print)
            {
                printf("query height int %d\n", query_height_int);
            }
            float query_height = (float) query_height_int / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

if (should_print)
                        {
                            printf("query_heigth %f\n", query_height);
                        }

            if (isnan(query_height) || query_height < min_height)
               continue;


            if (should_print)
            {
               printf("point is inside\n");
            }

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

if (should_print)
{
  printf("finished search for area \n");
  printf("running height total %f\n", running_height_total);
  printf("points inside polygon %d\n", points_inside_polygon);
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

        // TODO potentially use the z solution.
       snap_height += params[HEIGHT_OFFSET];

       write_imageui(steppable_map, key, (uint4)(VALID,0,0,0));
       write_imageui(snapped_height_map, key, (uint4)((int)(snap_height * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
       write_imageui(snapped_normal_x_map, key, (uint4)((int)(normal.x * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
       write_imageui(snapped_normal_y_map, key, (uint4)((int)(normal.y * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
       write_imageui(snapped_normal_z_map, key, (uint4)((int)(normal.z * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
    }
    else
    {
       write_imageui(steppable_map, key, (uint4)(SNAP_FAILED,0,0,0));
       write_imageui(snapped_height_map, key, (uint4)(0, 0, 0, 0));
       write_imageui(snapped_normal_x_map, key, (uint4)(0, 0, 0, 0));
       write_imageui(snapped_normal_y_map, key, (uint4)(0, 0, 0, 0));
       write_imageui(snapped_normal_z_map, key, (uint4)(0, 0, 0, 0));
    }
}
