#define FLOAT_TO_INT_SCALE 10000

#define centerX 0
#define centerY 1

#define RESOLUTION 0
#define CENTER_INDEX 1
#define SNAP_HEIGHT_THRESHOLD 2

int pointXOffsets[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
int pointYOffsets[9] = {1, 1, 1, 0, 0, 0, -1, -1, -1};

int get_x_idx(float x, float center, float* params)
{
    return coordinate_to_index(x, center, params[RESOLUTION], params[CENTER_INDEX]);
}

int get_y_idx(float y, float center, float* params)
{
    return coordinate_to_index(y, center, params[RESOLUTION], params[CENTER_INDEX]);
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
    float neg = matrix[2] * matrix[3] * matrix[6] + matrix[1] * matrix[3] * matrix[8] + matrix[0] * matrix[5] * matrix[7];

    return pos - neg;
}

float* invert3x3Matrix(float* matrix)
{
    float det = determinant3x3Matrix(matrix);
    float ret[9];

    ret[0] = (matrix[4] * matrix[8] - matrix[5] * matrix[7]) / det;
    ret[1] = (matrix[2] * matrix[7] - matrix[1] * matrix[8]) / det;
    ret[2] = (matrix[2] * matrix[5] - matrix[3] * matrix[4]) / det;
}

float* solveForPlaneCoefficients(float* covariance_matrix, float* z_variance_vector)
{
    float* inverse_covariance_matrix = invert3x3Matrix(covariance_matrix);
    float coefficients[3] = {0.0, 0.0, 0.0};
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            coefficients[row] += inverse_covariance_matrix[col + row * 3] * z_variance_vector[col];
        }
    }

    return coefficients;
}

void kernel computeSurfaceNormals(read_only float* params,
                                  read_only float* localization,
                                  read_only image2d_t height_map,
                                  write_only image2d_t normal_x_mat,
                                  write_only image2d_t normal_y_mat,
                                  write_only image2d_t normal_z_mat,
                                  write_only image2d_t sampled_height)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int center_index = params[CENTER_INDEX]
    int cells_per_side = 2 * centerIndex + 1;

    float x_coordinate = index_to_coordinate(idx_x, localization[centerX], params[resolution], center_index);
    float y_coordinate = index_to_coordinate(idx_y, localization[centerY], params[resolution], center_index);

    int idx = get_idx_in_layer(idx_x, idx_y, center_index);

    // compute the maximum height in the area of interest.
    float max_z = -INFINITY;
    for (int cell = 0; cell < 9; cell++)
    {
        int idx_x_to_poll = idx_x + pointXOffsets[cell];
        int idx_y_to_poll = idx_y + pointYOffsets[cell];

        // check to make sure the index is in frame
        if (idx_x_to_poll < 0 || idx_x_to_poll > cells_per_side)
            continue;
        if (idx_y_to_poll < 0 || idx_y_to_poll > cells_per_side)
            continue;

        int idx_to_poll = get_idx_in_layer(idx_x_to_poll, idx_y_to_poll, center_index);

        float z_coordinate = read_imagef(height_map, idx_to_poll).x;

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

    for (int cell = 0; cell < 9; cell++)
    {
        int idx_x_to_poll = idx_x + pointXOffsets[cell];
        int idx_y_to_poll = idx_y + pointYOffsets[cell];
        int idx_to_poll = get_idx_in_layer(idx_x_to_poll, idx_y_to_poll, center_index);

        if (idx_x_to_poll < 0 || idx_x_to_poll > cells_per_side)
            continue;
        if (idx_y_to_poll < 0 || idx_y_to_poll > cells_per_side)
            continue;

        float z_coordinate_to_poll = heightMap[idx_to_poll];

        if (isnan(z_coordinate_to_poll) || z_coordinate_to_poll < min_z)
            continue;


        float x_coordinate_to_poll = index_to_coordinate(idx_x_to_poll, localization[centerX], params[resolution], center_index);
        float y_coordinate_to_poll = index_to_coordinate(idx_y_to_poll, localization[centerY], params[resolution], center_index);

        n++;
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
        float coefficients = solveForPlaneCoefficients(covariance_matrix, z_variance_vector);

        float mag = sqrt(coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1] + 1.0);
        float x_solution = x / n;
        float y_solution = y / n;
        float z_solution = -coefficients[0] * x_solution - coefficients[1] * y_solution - coefficients[2];

        normal_x = coefficients[0] / mag;
        normal_y = coefficients[1] / mag;
        normal_z = 1.0 / mag;

        // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
        height_at_center = normal_x / normal_z * (x_solution - x_solution_to_poll) + normal_y / normal_z * (y_solution - y_solution_to_poll) + z_solution_to_poll;
    }
    else
    {
        normal_x = 0.0;
        normal_y = 0.0;
        normal_z = 1.0;

        height_at_center = nanf;
    }

    write_imagef(normal_x_mat, key, (float4)(normal_x, 0, 0, 0));
    write_imagef(normal_y_mat, key, (float4)(normal_y, 0, 0, 0));
    write_imagef(normal_z_mat, key, (float4)(normal_z, 0, 0, 0));
    write_imagef(sampled_height, key, (float4)(height_at_center, 0, 0, 0));
}




