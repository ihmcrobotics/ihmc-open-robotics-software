#define FLOAT_TO_INT_SCALE 10000

#define RESOLUTION 0
#define CENTER_INDEX 1
#define centerX 2
#define centerY 3
#define SNAP_HEIGHT_THRESHOLD 4
#define PATCH_WIDTH 5

#define MAX_INCLINE 0
#define COMPUTE_SURFACE_NORMAL_COST 1
#define NOMINAL_INCLINE 2
#define INCLINE_COST_WEIGHT 3
#define INCLINE_COST_DEADBAND 3

#define VALID -1
#define INVALID_SNAP 0
#define TOO_STEEP 1
#define STEP_TOO_HIGH 2
#define COLLISION 3
#define NON_TRAVERSIBLE 4

int key_to_x_index(int key, int center_index)
{
    return key % (2 * center_index + 1);
}

int key_to_y_index(int key, int center_index)
{
    return key / (2 * center_index + 1);
}

int indices_to_key(int x_index, int y_index, int centerIndex)
{
    return x_index + y_index * (2 * centerIndex + 1);
}

float index_to_coordinate(int index, float grid_center, float resolution, int center_index)
{
    return (index - center_index) * resolution + grid_center;
}

float determinant3x3Matrix(float* matrix)
{
    float pos = matrix[0] * matrix[4] * matrix[8] + matrix[1] * matrix[5] * matrix[6] + matrix[2] * matrix[3] * matrix[7];
    float neg = matrix[2] * matrix[4] * matrix[6] + matrix[1] * matrix[3] * matrix[8] + matrix[0] * matrix[5] * matrix[7];

    return pos - neg;
}

float* invert3x3Matrix(float* matrix)
{
    float det = determinant3x3Matrix(matrix);
    float ret[9];

    float detMinor00 = matrix[4] * matrix[8] - matrix[5] * matrix[7];
    float detMinor01 = matrix[3] * matrix[8] - matrix[5] * matrix[6];
    float detMinor02 = matrix[3] * matrix[7] - matrix[4] * matrix[6];

    float detMinor10 = matrix[1] * matrix[8] - matrix[2] * matrix[7];
    float detMinor11 = matrix[0] * matrix[8] - matrix[2] * matrix[6];
    float detMinor12 = matrix[0] * matrix[7] - matrix[1] * matrix[6];

    float detMinor20 = matrix[1] * matrix[5] - matrix[2] * matrix[4];
    float detMinor21 = matrix[0] * matrix[5] - matrix[2] * matrix[3];
    float detMinor22 = matrix[0] * matrix[4] - matrix[1] * matrix[3];

    ret[0] = detMinor00 / det;
    ret[1] = -detMinor10 / det;
    ret[2] = detMinor20 / det;

    ret[3] = -detMinor01 / det;
    ret[4] = detMinor11 / det;
    ret[5] = -detMinor21 / det;

    ret[6] = detMinor02 / det;
    ret[7] = -detMinor12 / det;
    ret[8] = detMinor22 / det;

    return ret;
}

float* solveForPlaneCoefficients(float* covariance_matrix, float* z_variance_vector)
{
    float* inverse_covariance_matrix = invert3x3Matrix(covariance_matrix);
    float coefficients[3] = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            coefficients[row] += inverse_covariance_matrix[col + row * 3] * z_variance_vector[col];
        }
    }

    return coefficients;
}

void kernel computeSurfaceNormalsWithLeastSquares(global float* params,
                                                  global int* offsets,
                                  global float* height_map,
                                  global float* normal_xyz_buffer,
                                  global float* sampled_height)
{
    int key = get_global_id(0);

    int center_index = (int) params[CENTER_INDEX];

    int idx_x = key_to_x_index(key, center_index);
    int idx_y = key_to_y_index(key, center_index);

    int cells_per_side = 2 * center_index + 1;

    int patch_cell_half_width = offsets[0];

    int connection_side_size = 2 * patch_cell_half_width + 1;
    int connections = connection_side_size * connection_side_size;

    // compute the maximum height in the area of interest.
    float max_z = -INFINITY;
    for (int cell = 0; cell < connections; cell++)
    {
        int idx_x_to_poll = idx_x + offsets[cell + 1];
        int idx_y_to_poll = idx_y + offsets[connections + cell + 1];

        // check to make sure the index is in frame
        if (idx_x_to_poll < 0 || idx_x_to_poll > cells_per_side)
            continue;
        if (idx_y_to_poll < 0 || idx_y_to_poll > cells_per_side)
            continue;

        int key_to_poll = indices_to_key(idx_x_to_poll, idx_y_to_poll, center_index);
        float z_coordinate = height_map[key_to_poll];

        if (!isnan(z_coordinate))
            max_z = max(max_z, z_coordinate);
    }

    // fit a plane to points in the patch, which are a three by three grid.
    float min_z = max_z - params[SNAP_HEIGHT_THRESHOLD];

    float n = 0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float xx = 0.0;
    float xy = 0.0;
    float xz = 0.0;
    float yy = 0.0;
    float yz = 0.0;
    float zz = 0.0;

    for (int cell = 0; cell < connections; cell++)
    {
        int idx_x_to_poll = idx_x + offsets[cell + 1];
        int idx_y_to_poll = idx_y + offsets[connections + cell + 1];

        if (idx_x_to_poll < 0 || idx_x_to_poll > cells_per_side)
            continue;
        if (idx_y_to_poll < 0 || idx_y_to_poll > cells_per_side)
            continue;

        int key_to_poll = indices_to_key(idx_x_to_poll, idx_y_to_poll, center_index);
        float z_coordinate_to_poll = height_map[key_to_poll];

        if (isnan(z_coordinate_to_poll) || z_coordinate_to_poll < min_z)
            continue;

        float x_coordinate_to_poll = index_to_coordinate(idx_x_to_poll, params[centerX], params[RESOLUTION], center_index);
        float y_coordinate_to_poll = index_to_coordinate(idx_y_to_poll, params[centerY], params[RESOLUTION], center_index);

        n += 1.0f;
        x += x_coordinate_to_poll;
        y += y_coordinate_to_poll;
        z += z_coordinate_to_poll;
        xx += x_coordinate_to_poll * x_coordinate_to_poll;
        xy += x_coordinate_to_poll * y_coordinate_to_poll;
        xz += x_coordinate_to_poll * z_coordinate_to_poll;
        yy += y_coordinate_to_poll * y_coordinate_to_poll;
        yz += y_coordinate_to_poll * z_coordinate_to_poll;
        zz += z_coordinate_to_poll * z_coordinate_to_poll;
    }

    float normal_x, normal_y, normal_z, height_at_center;

    if (n > 3)
    {
        float covariance_matrix[9] = {xx, xy, x, xy, yy, y, x, y, n};
        float z_variance_vector[3] = {-xz, -yz, -z};
        float* coefficients = solveForPlaneCoefficients(covariance_matrix, z_variance_vector);

        float x_solution = x / n;
        float y_solution = y / n;
        float z_solution = -coefficients[0] * x_solution - coefficients[1] * y_solution - coefficients[2];

        float mag = sqrt(coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1] + 1.0);
        normal_x = coefficients[0] / mag;
        normal_y = coefficients[1] / mag;
        normal_z = 1.0 / mag;

        float x_coordinate = index_to_coordinate(idx_x, params[centerX], params[RESOLUTION], center_index);
        float y_coordinate = index_to_coordinate(idx_y, params[centerY], params[RESOLUTION], center_index);
        float z_coordinate = height_map[key];

        // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
        height_at_center = normal_x / normal_z * (x_solution - x_coordinate) + normal_y / normal_z * (y_solution - y_coordinate) + z_coordinate;

        if (isnan(normal_x))
        {
            normal_x = 0.0;
            normal_y = 0.0;
            normal_z = 1.0;

            height_at_center = NAN;
        }
    }
    else
    {
        normal_x = 0.0;
        normal_y = 0.0;
        normal_z = 1.0;

        height_at_center = NAN;
    }

    normal_xyz_buffer[3 * key] = normal_x;
    normal_xyz_buffer[3 * key + 1] = normal_y;
    normal_xyz_buffer[3 * key + 2] = normal_z;
    sampled_height[key] = height_at_center;
}

void kernel snapVertices(global float* params,
                         global float* height_map,
                         global int* neighbor_offsets,
                         global float* snapped_average_vertex_height)
{
    int key = get_global_id(0);

    int center_index = (int) params[CENTER_INDEX];
    float resolution = params[RESOLUTION];

    int idx_x = key_to_x_index(key, center_index);
    int idx_y = key_to_y_index(key, center_index);

    int number_of_neighbors = neighbor_offsets[0];
    int cells_per_side = 2 * center_index + 1;

    // compute the max z
    float max_z = -INFINITY;
    for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
    {
        int neighbor_idx_x = idx_x + neighbor_offsets[1 + neighborIdx];
        int neighbor_idx_y = idx_y + neighbor_offsets[1 + number_of_neighbors + neighborIdx];

        if (neighbor_idx_x < 0 || neighbor_idx_x > cells_per_side)
        {
            continue;
        }
        if (neighbor_idx_y < 0 || neighbor_idx_y > cells_per_side)
        {
            continue;
        }

        int neighbor_key = indices_to_key(neighbor_idx_x, neighbor_idx_y, center_index);
        float neighbor_z = height_map[neighbor_key];

        if (!isnan(neighbor_z))
            max_z = max(max_z, neighbor_z);
    }

    float height_sample_delta = 0.08f;
    float min_z = max_z - height_sample_delta;

    // compute the average z height
    float running_sum = 0;
    int number_of_samples = 0;
    for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
    {
        int neighbor_idx_x = idx_x + neighbor_offsets[1 + neighborIdx];
        int neighbor_idx_y = idx_y + neighbor_offsets[1 + number_of_neighbors + neighborIdx];

        if (neighbor_idx_x < 0 || neighbor_idx_x > cells_per_side)
        {
            continue;
        }
        if (neighbor_idx_y < 0 || neighbor_idx_y > cells_per_side)
        {
            continue;
        }

        int neighbor_key = indices_to_key(neighbor_idx_x, neighbor_idx_y, center_index);
        float neighbor_z = height_map[neighbor_key];

        if (!isnan(neighbor_z) && neighbor_z > min_z)
        {
            running_sum += neighbor_z;
            number_of_samples++;
        }
    }

    snapped_average_vertex_height[key] = running_sum / number_of_samples;
}

void kernel computeEdges(global float* params,
                         global float* planner_params,
                         global int* neighbor_offsets,
                         global float* height_map,
                         global float* snapped_height_map,
                         global float* normal_xyz_buffer,
                         global int* edge_rejection_reason,
                         global float* delta_height_map,
                         global float* incline_map,
                         global float* incline_cost_map,
                         global float* roll_cost_map,
                         global float* edge_cost_map)
{
    int key = get_global_id(0);

    int center_index = (int) params[CENTER_INDEX];
    float resolution = params[RESOLUTION];

    int idx_x = key_to_x_index(key, center_index);
    int idx_y = key_to_y_index(key, center_index);

    int number_of_neighbors = neighbor_offsets[0];

    // if the current snapped height is bad, invalidate all the edges and return
    float snapped_height = snapped_height_map[key];
    if (isnan(snapped_height))
    {
         for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
         {
             int edge_key = number_of_neighbors * key + neighborIdx;
             edge_rejection_reason[edge_key] = INVALID_SNAP;
         }
         return;
    }

    float start_x = index_to_coordinate(idx_x, params[centerX], resolution, center_index);
    float start_y = index_to_coordinate(idx_y, params[centerY], resolution, center_index);

    int cells_per_side = 2 * center_index + 1;

    for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
    {
        int neighbor_idx_x = idx_x + neighbor_offsets[1 + neighborIdx];
        int neighbor_idx_y = idx_y + neighbor_offsets[1 + number_of_neighbors + neighborIdx];
        int edge_key = number_of_neighbors * key + neighborIdx;

        if (neighbor_idx_x < 0 || neighbor_idx_x > cells_per_side)
        {
            edge_rejection_reason[edge_key] = INVALID_SNAP;
            continue;
        }
        if (neighbor_idx_y < 0 || neighbor_idx_y > cells_per_side)
        {
            edge_rejection_reason[edge_key] = INVALID_SNAP;
            continue;
        }

        int neighbor_key = indices_to_key(neighbor_idx_x, neighbor_idx_y, center_index);
        float snapped_neighbor_height = snapped_height_map[neighbor_key];
        if (isnan(snapped_neighbor_height))
        {
            edge_rejection_reason[edge_key] = INVALID_SNAP;
            continue;
        }

        float neighbor_x = index_to_coordinate(neighbor_idx_x, params[centerX], resolution, center_index);
        float neighbor_y = index_to_coordinate(neighbor_idx_y, params[centerY], resolution, center_index);
        float xy_distance = sqrt((neighbor_x - start_x) * (neighbor_x - start_x) + (neighbor_y - start_y) * (neighbor_y - start_y));

        float delta_height = fabs(snapped_neighbor_height - snapped_height);
        float incline = atan2(delta_height, xy_distance);

        delta_height_map[edge_key] = delta_height;
        incline_map[edge_key] = incline;

        if (fabs(incline) > planner_params[MAX_INCLINE])
        {
            edge_rejection_reason[edge_key] = TOO_STEEP;
            continue;
        }

        // TODO check for collisions

        // TODO check for traversibility

        float edge_cost = xy_distance;
        float incline_cost = 0.0;
        if (incline > planner_params[NOMINAL_INCLINE])
        {
            float inclineDelta = fabs( (incline - planner_params[NOMINAL_INCLINE]));
            incline_cost = planner_params[INCLINE_COST_WEIGHT] * (max(0.0f, inclineDelta - planner_params[INCLINE_COST_DEADBAND]));
        }
        edge_cost += incline_cost;

        // TODO compute roll cost

        // TODO compute traversibility cost

        edge_cost_map[edge_key] = edge_cost;
        incline_cost_map[edge_key] = incline_cost;
    }
}


