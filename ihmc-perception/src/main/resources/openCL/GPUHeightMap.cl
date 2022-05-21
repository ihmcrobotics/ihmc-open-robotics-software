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

#define WIDTH 0
#define HEIGHT 1
#define MIN_VALID_DISTANCE 2
#define RAMPED_HEIGHT_RANGE_A 3
#define RAMPED_HEIGHT_RANGE_B 4
#define RAMPED_HEIGHT_RANGE_C 5
#define MAX_HEIGHT_RANGE 6
#define MAHALONBIS_THRES 7
#define OUTLIER_VARIANCE 8
#define TRAVERSABILITY_INLIER 9
#define SENSOR_NOISE_FACTOR 10

#define HEIGHT_LAYER 0
#define VARIANCE_LAYER 1
#define VALIDITY_LAYER 2
#define TRAVERSABILITY_LAYER 3
#define TIME_LAYER 4
#define UPPER_BOUND_LAYER 5
#define IS_UPPER_BOUND_LAYER 6


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

float z_noise(float z, global float* params)
{
    return params[SENSOR_NOISE_FACTOR] * z * z;
}

int get_idx(float x, float y, float center_x, float center_y, global float* params)
{
    int idx_x = clamp(get_x_idx(x, center_x), 0, params[WIDTH] - 1);
    int idx_y = clamp(get_y_idx(y, center_y), 0, parameters[HEIGHT] - 1);

    return params[WIDTH] * idx_x + idx_y;
}

int get_map_idx(int idx, int layer_n, global float* params)
{
    const int layer = params[WIDTH] * params[HEIGHT];
    return layer * layer_n + idx;
}


float point_sensor_distance(float3 x, float y, float z, float sx, float sy, float sz)
{
    float d = (x - sx) * (x - sx) + (y - sy) * (y - sy) + (z - sz) * (z - sz);
    return d;
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

float transform_p(float x, float y, float z, float r0, float r1, float r2, float t)
{
    return r0 * x + r1 * y + r2 * z + t;
}

 void kernel addPointsKernel(global float* points_in, read_only float* localization, global float* params, global float* map, global float* newMap)
 {
    int i = get_global_id(0);

    float rx = points_in[i * 3];
    float ry = points_in[i * 3 + 1];
    float rz = points_in[i * 3 + 2];

    // TODO make this a float3 operation
    float x = transform_p(rx, ry, rz, localization[r00], localization[r01], localization[r02], localization[tx]);
    float y = transform_p(rx, ry, rz, localization[r10], localization[r11], localization[r12], localization[ty]);
    float z = transform_p(rx, ry, rz, localization[r20], localization[r21], localization[r22], localization[tz]);

    float v = z_noise(rz);

    if (is_valid(x, y, z, localization[tx], localization[ty], localization[tz], params))
    {
        int idx = get_idx(x, y, localization[centerX], localization[centerY], params);
        int idx_x = get_x_idx(x, y, localization[centerX], localization[centerY], params);
        int idx_y = get_y_idx(x, y, localization[centerX], localization[centerY], params);

        if (is_inside(idx_x, idx_y, params))
        {
            float map_h = map[get_map_idx(idx, HEIGHT_LAYER)];
            float map_v = map[get_map_idx(idx, VARIANCE_LAYER)];
            float num_points = newMap[get_map_idx(idx, TIME_LAYER)]; // what?

            if (abs(map_h - z) > (map_v * params[MAHALONBIS_THRES]))
            {
                atomic_add(&map[get_map_idx(idx, VARIANCE_LAYER)], params[OUTLIER_VARIANCE]);
            }
            else
            {
                float new_height = (map_h * v + z * map_v) / (map_v + v);
                float new_variance = (map_v * v) / (map_v + v);

                atomic_add(&newMap[get_map_idx(idx, HEIGHT_LAYER)], new_h);
                atomic_add(&newMap[get_map_idx(idx, VARIANCE_LAYER)], new_v);
                atomic_add(&newMap[get_map_idx(idx, VALIDITY_LAYER)], new_h);

                // is Valid
                map[get_map_idx(idx, VALIDITY_LAYER)] = 1;

                // Time layer
                map[get_map_idx(idx, TIME_LAYER)] = 0.0;
                // Upper bound
                map[get_map_idx(idx, UPPER_BOUND_LAYER)] = new_h;
                map[get_map_idx(idx, IS_UPPER_BOUND_LAYER)] = 0.0;

            }
            // visibility cleanup
        }
    }
}

void kernel errorCountingKernel(global float* map, global float* points_in, global float* params,
                                global float* newMap, global float* error, global float* error_cnt)
{
    int i = get_global_id(0);

    float rx = points_in[i * 3];
    float ry = points_in[i * 3 + 1];
    float rz = points_in[i * 3 + 2];

    // TODO make this a float3 operation
    float x = transform_p(rx, ry, rz, localization[r00], localization[r01], localization[r02], localization[tx]);
    float y = transform_p(rx, ry, rz, localization[r10], localization[r11], localization[r12], localization[ty]);
    float z = transform_p(rx, ry, rz, localization[r20], localization[r21], localization[r22], localization[tz]);

    float v = z_noise(rz);

    if (!is_valid(x, y, z, localization[tx], localization[ty], localization[tz], params))
    {
        return;
    }

    int idx = get_idx(x, y, localization[centerX], localization[centerY]);
    if (!is_inside(idx))
    {
        return;
    }

    float map_h = map[get_map_idx(idx, HEIGHT_LAYER)];
    float map_v = map[get_map_idx(idx, VARIANCE_LAYER)];
    float map_valid = map[get_map_idx(idx, VALIDITY_LAYER)];
    float map_t = map[get_map_idx(idx, TRANVERSABILITY_LAYER)];

    float error_h = z - map_h;
    if (map_valid > 0.5 && (abs(error_h) < (map_v * params[MAHALANOBIS_THRESH]))
        && map_v < params[OUTLIER_VARIANCE] / 2.0
        && map_t > params[TRAVERSABILITY_INLIER])
    {
        float error_h = z - map_h;
        atomic_add(&error[0], error_h);
        atomic_add(&error_cnt[0], 1);
        atomic_add(&newMap[get_map_idx(idx, TRAVERSABILITY_LAYER)], 1.0);
    }

    atomic_add(&newMap[get_map_idx(idx, TIME_LAYER)], 1.0);
}

