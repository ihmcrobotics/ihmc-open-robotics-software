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

#define DEPTH_CX 0
#define DEPTH_CY 1
#define DEPTH_FX 2
#define DEPTH_FY 3

int clamp_val(int x, int min_x, int max_x)
{
    return max(min(x, max_x), min_x);
}

float z_noise(float z, read_only float* params)
{
    return params[SENSOR_NOISE_FACTOR] * z * z;
}

float rotate_point(float3 point, float3 r)
{
    return dot(point, r);
}

float3 transform_point(float3 point, float3 rx, float3 ry, float3 rz, float3 translation)
{
    float3 rotatedPoint = (float3) (0.0, 0.0, 0.0);
    rotatedPoint.x = rotate_point(point, rx);
    rotatedPoint.y = rotate_point(point, ry);
    rotatedPoint.z = rotate_point(point, rz);
    return rotatedPoint + translation;
}

float point_sensor_distance(float3 point, float3 sensor)
{
    float3 delta = point - sensor;

    return dot(delta, delta);
}

int get_x_idx(float x, float center, read_only float* params)
{
    int i = (x - center)  / params[RESOLUTION] + 0.5 * params[WIDTH];
    return clamp_val(i, 0, params[WIDTH] - 1);
}

int get_y_idx(float y, float center, read_only float* params)
{
    int i = (y - center) / params[RESOLUTION] + 0.5 * params[HEIGHT];

    return clamp_val(i, 0, params[HEIGHT] - 1);
}

int get_idx_in_layer(int idx_x, int idx_y, read_only float* params)
{
    return params[WIDTH] * idx_x + idx_y;
}

int get_idx(float x, float y, float center_x, float center_y, read_only float* params)
{
    int idx_x = get_x_idx(x, center_x, params);
    int idx_y = get_y_idx(y, center_y, params);

    return get_idx_in_layer(idx_x, idx_y, params);
}

int get_map_idx_in_layer(int idx_x, int idx_y, int layer_n, read_only float* params)
{
    const int layer = params[WIDTH] * params[HEIGHT];
    return layer * layer_n + get_idx_in_layer(idx_x, idx_y, params);
}

int get_map_idx(int idx, int layer_n, read_only float* params)
{
  //  if (idx == 0) printf("layer_n %d \n", layer_n);
    const int layer = params[WIDTH] * params[HEIGHT];
    return layer * layer_n + idx;
}


bool is_inside(int idx_x, int idx_y, read_only float* params)
{
    int max_width = params[WIDTH];
    if (idx_x == -1 || idx_x == max_width)
    {
        return false;
    }
    int max_height = params[HEIGHT];
    if (idx_y == -1 || idx_y == max_height)
    {
        return false;
    }

   // printf("Actually inside! %i, (%i, %i)\n", idx, idx_x, idx_y);
    return true;
}

bool is_valid(float3 point, float3 sensor, read_only float* params)
{
    float d = point_sensor_distance(point, sensor);
    float min = 0.0;

    if (d < params[MIN_VALID_DISTANCE] * params[MIN_VALID_DISTANCE])
    {
        //printf("Not valid from distance");
        return false;
    }

    return true;
    //float dxy = max(sqrt(x * x + y * y) - params[RAMPED_HEIGHT_RANGE_B], min);

    //if (z - sz > dxy * params[RAMPED_HEIGHT_RANGE_A] + params[RAMPED_HEIGHT_RANGE_C] || z - sz > params[MAX_HEIGHT_RANGE])
    //{
        //printf("Not valid from ramp");
    //    return false;
    //}
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


void kernel zeroValuesKernel(global float* params, global float* newMap)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int height_idx = get_map_idx_in_layer(idx_x, idx_y, HEIGHT_LAYER, params);
    int variance_idx = get_map_idx_in_layer(idx_x, idx_y, VARIANCE_LAYER, params);
    int counter_idx = get_map_idx_in_layer(idx_x, idx_y, POINT_COUNTER_LAYER, params);

    newMap[height_idx] = 0.0;
    newMap[variance_idx] = 0.0;
    newMap[counter_idx] = 0.0;
}

float3 back_project_to_image_frame(int2 pos, float Z, global float* params)
{
   float px = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
   float py = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
   return (float3) (px, py, Z);
}

float3 project_from_image_frame_to_camera_frame(float3 point_in_image_frame)
{
    return (float3) (point_in_image_frame.z, -point_in_image_frame.x, point_in_image_frame.y);
}

float3 get_point_from_image(read_only image2d_t depth_image, int x, int y, global float* params)
{
    int2 key = (int2)(x,y);
    float Z = ((float)read_imageui(depth_image, key).x)/(float) 1000;

    if (Z > 0.1f)
    {
        return project_from_image_frame_to_camera_frame(back_project_to_image_frame(key, Z, params));
    }
    else
    {
        return (float3) (0.0, 0.0, 0.0);
    }
}

void kernel addPointsKernel(global float* points_in, global float* localization, global float* params,  global float* newMap)
{
    int i = get_global_id(0);

    float3 point = (float3) (points_in[i * 3], points_in[i * 3 + 1], points_in[i * 3 + 2]);
    float3 sensor = (float3) (localization[tx], localization[ty], localization[tz]);
    float3 rotation_x = (float3) (localization[r00], localization[r01], localization[r02]);
    float3 rotation_y = (float3) (localization[r10], localization[r11], localization[r12]);
    float3 rotation_z = (float3) (localization[r20], localization[r21], localization[r22]);

    float3 pointInWorld = transform_point(point, rotation_x, rotation_y, rotation_z, sensor);

    if (is_valid(pointInWorld, sensor, params))
    {
        int idx_x = get_x_idx(pointInWorld.x, localization[centerX], params);
        int idx_y = get_y_idx(pointInWorld.y, localization[centerY], params);
        int idx = get_idx_in_layer(idx_x, idx_y, params);

        // TODO pretty sure this is always true by construction
        if (is_inside(idx_x, idx_y, params))
        {
            int height_idx = get_map_idx(idx, HEIGHT_LAYER, params);
            int variance_idx = get_map_idx(idx, VARIANCE_LAYER, params);
            int counter_idx = get_map_idx(idx, POINT_COUNTER_LAYER, params);

            float new_height = pointInWorld.z;
            float new_variance = new_height * new_height;

            AtomicAdd_g_f(&newMap[height_idx], new_height);
            AtomicAdd_g_f(&newMap[variance_idx], new_variance);
            AtomicAdd_g_f(&newMap[counter_idx], 1.0);


            // visibility cleanup
        }
    }
}

void kernel addPointsFromImageKernel(read_only image2d_t depth_image, global float* localization, global float* params,  global float* intrinsics, global float* newMap)
{
    int image_y = get_global_id(0);
    int image_x = get_global_id(1);

    float3 point_in_camera_frame = get_point_from_image(depth_image, image_x, image_y, intrinsics);
    float3 sensor = (float3) (localization[tx], localization[ty], localization[tz]);
    float3 rx = (float3) (localization[r00], localization[r01], localization[r02]);
    float3 ry = (float3) (localization[r10], localization[r11], localization[r12]);
    float3 rz = (float3) (localization[r20], localization[r21], localization[r22]);

    float3 pointInWorld = transform_point(point_in_camera_frame, rx, ry, rz, sensor);

    if (is_valid(pointInWorld, sensor, params))
    {
        int idx_x = get_x_idx(pointInWorld.x, localization[centerX], params);
        int idx_y = get_y_idx(pointInWorld.y, localization[centerY], params);
        int idx = get_idx_in_layer(idx_x, idx_y, params);

        // TODO pretty sure this is always true by construction
 //       if (is_inside(idx_x, idx_y, params))
 //       {
            int height_idx = get_map_idx(idx, HEIGHT_LAYER, params);
            int variance_idx = get_map_idx(idx, VARIANCE_LAYER, params);
            int counter_idx = get_map_idx(idx, POINT_COUNTER_LAYER, params);


            float new_height = pointInWorld.z;
            float new_variance = new_height * new_height;

            AtomicAdd_g_f(&newMap[height_idx], new_height);
            AtomicAdd_g_f(&newMap[variance_idx], new_variance);
            AtomicAdd_g_f(&newMap[counter_idx], 1.0);

            // visibility cleanup
 //       }
    }
}

void kernel averageMapKernel(global float* newMap, global float* params)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int height_idx = get_map_idx_in_layer(idx_x, idx_y, HEIGHT_LAYER, params);
    int variance_idx = get_map_idx_in_layer(idx_x, idx_y, VARIANCE_LAYER, params);

    float new_h = newMap[height_idx];
    float new_v = newMap[variance_idx];
    float new_cnt = newMap[get_map_idx_in_layer(idx_x, idx_y, POINT_COUNTER_LAYER, params)];

    if (new_cnt > 0)
    {
        newMap[height_idx] = new_h / new_cnt;
        newMap[variance_idx] = (new_v - new_h * new_h / new_cnt) / (new_cnt - 1);
    }
}





