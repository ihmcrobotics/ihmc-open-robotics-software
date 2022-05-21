 #define centerX 1
 #define centerY 2
 #define r00 2
 #define r01 3
 #define r02 4
 #define r10 5
 #define r11 6
 #define r12 7
 #define r20 8
 #define r21 9
 #define r22 10
 #define tx 11
 #define ty 12
 #define tz 13

 #define WIDTH = 0
 #define HEIGHT = 0


 void kernel addPointsKernel(read_only float* points_in, read_only float* localization, global float* params)
 {
    int i = get_global_id(0);

    float rx = points_in[i * 3];
    float ry = points_in[i * 3 + 1];
    float rz = points_in[i * 3 + 2];

    float x = transform_p(rx, ry, rz, localization[r00], localization[r01], localization[r02], localization[tx]);
    float y = transform_p(rx, ry, rz, localization[r10], localization[r11], localization[r12], localization[ty]);
    float z = transform_p(rx, ry, rz, localization[r20], localization[r21], localization[r22], localization[tz]);

    // TODO v = z_noise(rz);

    if (is_valid(x, y, z, localization[tx], localization[ty], localization[tz], params))
    {
        int idx = get_idx(x, y, localization[centerX], localization[centerY], params);
        if (is_inside(idx, params))
        {
            // TODO start to populate the map
        }
    }

}

float transform_p(float x, float y, float z, float r0, float r1, float r2, float t)
{
    return r0 * x + r1 * y + r2 * z + t;
}

float clamp(float x, float min_x, float max_x)
{
    return max(min(x, max_x), min_x);
}

int get_x_idx(float x, float center, global float* params)
{
    int i = (x - center) / params[RESOLUTION] + 0.5 * params[WIDTH];
    return i;
}

int get_y_idx(float y, float center, global float* params)
{
    int i = (y - center) / params[RESOLUTION] + 0.5 * params[HEIGHT];
    return i;
}

int get_idx(float x, float y, float center_x, float center_y, global float* params)
{
    int idx_x = clamp(get_x_idx(x, center_x), 0, params[WIDTH] - 1);
    int idx_y = clamp(get_y_idx(y, center_y), 0, parameters[HEIGHT] - 1);

    return params[WIDTH] * idx_x + idx_y;
}

bool is_inside(int idx, global float* params)
{
    int idx_x = idx / params[WIDTH];
    int idx_y = idx % params[WIDTH];

    if (idx_x == 0 || idx_x == WIDTH - 1)
    {
        return false;
    }
    if (idx_y == 0 || idx_y == HEIGHT - 1)
    {
        return false;
    }

    return true;
}

bool is_valid(float x, float y, float z, float sx, float sy, float sz, global float* params)
{
    float d = point_sensor_distance(x, y, z, sx, sy, sz);
    float dxy = max(sqrt(x * x + y * y) - params[RAMPED_HEIGHT_RANGE_B], 0.0);
    if (d < params[MIN_VALID_DISTANCE] * params[MIN_VALID_DISTANCE])
    {
        return false;
    }
    else if (z - sz > dxy * params[RAMPED_HEIGHT_RANGE_A] + params[RAMPED_HEIGHT_RANGE_C] || z - sz > params[MAX_HEIGHT_RANGE])
    {
        return false;
    }

    return true;
}

float point_sensor_distance(float x, float y, float z, float sx, float sy, float sz)
{
    float d = (x - sx) * (x - sx) + (y - sy) * (y - sy) + (z - sz) * (z - sz);
    return d;
}
