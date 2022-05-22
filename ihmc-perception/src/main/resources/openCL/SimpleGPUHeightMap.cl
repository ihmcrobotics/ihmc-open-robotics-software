 #define centerX 0
 #define centerY 1
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
#define MAHALANOBIS_THRESH 7
#define OUTLIER_VARIANCE 8
#define TRAVERSABILITY_INLIER 9
#define SENSOR_NOISE_FACTOR 10
#define RESOLUTION 11
#define INITIAL_VARIANCE 12
#define MAX_VARIANCE 13

#define HEIGHT_LAYER 0
#define VARIANCE_LAYER 1
#define POINT_COUNTER_LAYER 2

int clamp_val(int x, int min_x, int max_x)
{
    //float clamped_upper = min(x, max_x);
    //float clamped_lower = max(clamed_upper, min_x);
    //return 0.0;
    return max(min(x, max_x), min_x);
}

float z_noise(float z, global float* params)
{
    return params[SENSOR_NOISE_FACTOR] * z * z;
}

float transform_p(float x, float y, float z, float r0, float r1, float r2, float t)
{
    return r0 * x + r1 * y + r2 * z + t;
}

// TODO switch this to use float3
float point_sensor_distance(float x, float y, float z, float sx, float sy, float sz)
{
    float d = (x - sx) * (x - sx);// + (y - sy) * (y - sy) + (z - sz) * (z - sz);
    return d;
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
    int idx_x = clamp_val(get_x_idx(x, center_x, params), 0, params[WIDTH] - 1);
    int idx_y = clamp_val(get_y_idx(y, center_y, params), 0, params[HEIGHT] - 1);

    return params[WIDTH] * idx_x + idx_y;
}

int get_map_idx(int idx, int layer_n, global float* params)
{
    const int layer = params[WIDTH] * params[HEIGHT];
    return layer * layer_n + idx;
}


bool is_inside(int idx, global float* params)
{
    int width = params[WIDTH];
    int idx_x = idx / width;
    int idx_y = idx % width;
    if (idx_x == 0 || idx_x == width - 1)
    {
        return false;
    }
    if (idx_y == 0 || idx_y == params[HEIGHT] - 1)
    {
        return false;
    }

    return true;
}

bool is_valid(float x, float y, float z, float sx, float sy, float sz, global float* params)
{
    float d = point_sensor_distance(x, y, z, sx, sy, sz);
    float min = 0.0;

    if (d < params[MIN_VALID_DISTANCE] * params[MIN_VALID_DISTANCE])
    {
        return false;
    }

    float dxy = max(sqrt(x * x + y * y) - params[RAMPED_HEIGHT_RANGE_B], min);

    if (z - sz > dxy * params[RAMPED_HEIGHT_RANGE_A] + params[RAMPED_HEIGHT_RANGE_C] || z - sz > params[MAX_HEIGHT_RANGE])
    {
        return false;
    }

    return true;
}

// taken from http://suhorukov.blogspot.com/2011/12/opencl-11-atomic-operations-on-floating.html
void AtomicAdd_g_f(volatile __global float *source, const float operand)
{
    union
    {
        unsigned int intVal;
        float floatVal;
    } newVal;
    union
    {
        unsigned int intVal;
        float floatVal;
    } prevVal;

    do
    {
        prevVal.floatVal = *source;
        newVal.floatVal = prevVal.floatVal + operand;
    }
    while (atomic_cmpxchg((volatile __global unsigned int *)source, prevVal.intVal, newVal.intVal) != prevVal.intVal);
}


void kernel addPointsKernel(global float* points_in, read_only float* localization, global float* params, global float* map, global float* newMap)
{
    int i = get_global_id(0);

    float rx = points_in[i * 3];
    float ry = points_in[i * 3 + 1];
    float rz = points_in[i * 3 + 2];

    float x = transform_p(rx, ry, rz, localization[r00], localization[r01], localization[r02], localization[tx]);
    float y = transform_p(rx, ry, rz, localization[r10], localization[r11], localization[r12], localization[ty]);
    float z = transform_p(rx, ry, rz, localization[r20], localization[r21], localization[r22], localization[tz]);

    float v = z_noise(rz, params);

    if (is_valid(x, y, z, localization[tx], localization[ty], localization[tz], params))
    {
        int idx = get_idx(x, y, localization[centerX], localization[centerY], params);

        if (is_inside(idx, params))
        {
            float map_h = map[get_map_idx(idx, HEIGHT_LAYER, params)];
            float map_v = map[get_map_idx(idx, VARIANCE_LAYER, params)];

                float new_height = (map_h * v + z * map_v) / (map_v + v);
                float new_variance = (map_v * v) / (map_v + v);

                AtomicAdd_g_f(&newMap[get_map_idx(idx, HEIGHT_LAYER, params)], new_height);
                AtomicAdd_g_f(&newMap[get_map_idx(idx, VARIANCE_LAYER, params)], new_variance);
                AtomicAdd_g_f(&newMap[get_map_idx(idx, POINT_COUNTER_LAYER, params)], 1.0);


            // visibility cleanup
        }
    }
}

// TODO restructure this to run i x j on image 2d
void kernel averageMapKernel(global float* newMap, global float* map, global float* params)
{
    int idx = get_global_id(0);

    float valid = map[get_map_idx(idx, VALIDITY_LAYER, params)];

    float new_h = newMap[get_map_idx(idx, HEIGHT_LAYER, params)];
    float new_v = newMap[get_map_idx(idx, VARIANCE_LAYER, params)];
    float new_cnt = newMap[get_map_idx(idx, VALIDITY_LAYER, params)];

    if (new_cnt > 0)
    {
        // too much variance, say it's not valid
        if (new_v / new_cnt > params[MAX_VARIANCE])
        {
            map[get_map_idx(idx, HEIGHT_LAYER, params)] = 0;
            map[get_map_idx(idx, VARIANCE_LAYER, params)] = params[INITIAL_VARIANCE];
        }
        else
        {
            map[get_map_idx(idx, HEIGHT_LAYER, params)] = new_h / new_cnt;
            map[get_map_idx(idx, VARIANCE_LAYER, params)] = new_v / new_cnt;
        }
    }

    if (valid < 0.5)
    {
        map[get_map_idx(idx, HEIGHT_LAYER, params)] = 0.0;
        map[get_map_idx(idx, VARIANCE_LAYER, params)] = params[INITIAL_VARIANCE];
    }
}




