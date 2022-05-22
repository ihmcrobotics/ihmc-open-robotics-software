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
#define RESOLUTION 2
#define MIN_VALID_DISTANCE 3
#define MAX_HEIGHT_RANGE 4
#define RAMPED_HEIGHT_RANGE_A 5
#define RAMPED_HEIGHT_RANGE_B 6
#define RAMPED_HEIGHT_RANGE_C 7
#define SENSOR_NOISE_FACTOR 8
#define INITIAL_VARIANCE 9
#define MAX_VARIANCE 10

#define HEIGHT_LAYER 0
#define VARIANCE_LAYER 1
#define POINT_COUNTER_LAYER 2

int clamp_val(int x, int min_x, int max_x)
{
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
    return clamp_val(i, 0, params[WIDTH] - 1);
}

int get_y_idx(float y, float center, global float* params)
{
    int i = (y - center) / params[RESOLUTION] + 0.5 * params[HEIGHT];
    return clamp_val(i, 0, params[HEIGHT] - 1);
}

int get_idx(float x, float y, float center_x, float center_y, global float* params)
{
    int idx_x = get_x_idx(x, center_x, params);
    int idx_y = get_y_idx(y, center_y, params);

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

bool is_inside(int idx_x, int idx_y, global float* params)
{
    if (idx_x == 0 || idx_x == params[WIDTH] - 1)
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

void AtomicAdd_g_ui(volatile __global float *source, const uint operand)
{
    union
    {
        unsigned int addressVal;
        uint intVal;
    } newVal;
    union
    {
        unsigned int addressVal;
        uint intVal;
    } prevVal;

    do
    {
        prevVal.intVal = *source;
        newVal.intVal = prevVal.intVal + operand;
    }
    while (atomic_cmpxchg((volatile __global unsigned int *)source, prevVal.addressVal, newVal.addressVal) != prevVal.addressVal);
}


void kernel addPointsKernel(read_only float* points_in,
                            read_only float* localization,
                            read_only float* params,
                            read_only image2d_t mapHeight,
                            read_only image2d_t mapVariance,
                            read_write image2d_t newMapHeight,
                            read_write image2d_t newMapVariance,
                            read_write image2d_t newMapCounter)
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
        int idx_x = get_x_idx(x, center_x, params);
        int idx_y = get_y_idx(y, center_y, params);

        int2 key = (int2)(x,y);

        if (is_inside(idx_x, idx_y, params))
        {
            float map_h = read_imagef(mapHeight, key).x;
            float map_v = read_imagef(mapVariance, key).x;

            float new_height = (map_h * v + z * map_v) / (map_v + v);
            float new_variance = (map_v * v) / (map_v + v);

            AtomicAdd_g_f(&read_imagef(newMapHeight, key).x, new_height);
            AtomicAdd_g_f(&read_imagef(newMapVariance, key).x, new_variance);
            AtomicAdd_g_ui(&read_imageui(newMapCounter, key).x, 1);

            // visibility cleanup
        }
    }
}

void kernel averageMapKernel(read_only image2d_t newMapHeight,
                             read_only image2d_t newMapVariance,
                             read_only image2d_t newMapCounter,
                             read_only float* params,
                             write_only image2d_t mapHeight,
                             write_only image2d_t mapVariance)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int2 key = (int2)(idx_x, idx_y);

    float new_h = read_imagef(newMapHeight, key).x;
    float new_v = read_imagef(newMapVariance, key).x;
    uint new_cnt = read_imageui(newMapCounter, key).x;

    if (new_cnt > 0)
    {
        // too much variance, say it's not valid
        if (new_v / new_cnt > params[MAX_VARIANCE])
        {
            write_imagef(mapHeight, key, (float4)(0,0,0,0));
            write_imagef(mapVariance, key, (float4)(params[INITIAL_VARIANCE],0,0,0));
        }
        else
        {
            write_imagef(mapHeight, key, (float4)(new_h / new_cnt,0,0,0));
            write_imagef(mapVariance, key, (float4)(new_v / new_cnt,0,0,0));
        }
    }
}




