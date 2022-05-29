#define FLOAT_TO_INT_SCALE 10000

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
    int ret = clamp_val(i, 0, params[WIDTH] - 1);

    if (ret >= params[WIDTH] - 5)
    {
        printf("On the far right side %i.\n", ret);
    }
    if (ret <= 5)
    {
        printf("On the far left side %i.\n", ret);
    }
    return ret;
}

int get_y_idx(float y, float center, read_only float* params)
{
    int i = (y - center) / params[RESOLUTION] + 0.5 * params[HEIGHT];
    int ret = clamp_val(i, 0, params[HEIGHT] - 1);;
    if (ret != i)
    {
        printf("Values not equal. %i, %i\n", i, ret);
    }

        if (ret >= params[HEIGHT] - 5)
        {
            printf("Far away side %i.\n", ret);
        }
        if (ret <= 5)
        {
            printf("Super close side %i.\n", ret);
        }
    return ret;
}

int get_idx_in_layer(int idx_x, int idx_y, read_only float* params)
{
    // assumes row major
    return params[WIDTH] * idx_x + idx_y;
}

int get_idx(float x, float y, float center_x, float center_y, read_only float* params)
{
    int idx_x = get_x_idx(x, center_x, params);
    int idx_y = get_y_idx(y, center_y, params);

    return get_idx_in_layer(idx_x, idx_y, params);
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

    return true;
}

bool is_valid(float3 point, float3 sensor, read_only float* params)
{
    float d = point_sensor_distance(point, sensor);
    float min = 0.0;

    if (d < params[MIN_VALID_DISTANCE] * params[MIN_VALID_DISTANCE])
    {
        printf("Returning that a point is not valid\n");
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

void kernel zeroValuesKernel(global float* params,
                             global int* centroid_data,
                             global int* variance_data,
                             global int* counter_data)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int idx = get_idx_in_layer(idx_x, idx_y, params);

    centroid_data[3 * idx] = 0;
    centroid_data[3 * idx + 1] = 0;
    centroid_data[3 * idx + 2] = 0;
    variance_data[3 * idx] = 0;
    variance_data[3 * idx + 1] = 0;
    variance_data[3 * idx + 2] = 0;
    counter_data[idx] = 0;
}

void kernel addPointsFromImageKernel(read_only image2d_t depth_image,
                                     global float* localization,
                                     global float* params,
                                     global float* intrinsics,
                                     global int* centroid_data,
                                     global int* variance_data,
                                     global int* counter_data)
{
    // recall that the pixels are bottom left, and go width to height
    int image_y = get_global_id(0);
    int image_x = get_global_id(1);

    float3 point_in_camera_frame = get_point_from_image(depth_image, image_x, image_y, intrinsics);
    float3 sensor = (float3) (localization[tx], localization[ty], localization[tz]);
    float3 rx = (float3) (localization[r00], localization[r01], localization[r02]);
    float3 ry = (float3) (localization[r10], localization[r11], localization[r12]);
    float3 rz = (float3) (localization[r20], localization[r21], localization[r22]);

    float3 pointInWorld = transform_point(point_in_camera_frame, rx, ry, rz, sensor);

  //  if (is_valid(pointInWorld, sensor, params))
    {
        int idx_x = get_x_idx(pointInWorld.x, localization[centerX], params);
        int idx_y = get_y_idx(pointInWorld.y, localization[centerY], params);
        // row major index
        int idx = get_idx_in_layer(idx_x, idx_y, params);

        // TODO pretty sure this is always true by construction
 //       if (is_inside(idx_x, idx_y, params))
 //       {
            float new_variance_x = pointInWorld.x * pointInWorld.x;
            float new_variance_y = pointInWorld.y * pointInWorld.y;
            float new_variance_z = pointInWorld.z * pointInWorld.z;

            int variance_x = (int) (new_variance_x * FLOAT_TO_INT_SCALE);
            int variance_y = (int) (new_variance_y * FLOAT_TO_INT_SCALE);
            int variance_z = (int) (new_variance_z * FLOAT_TO_INT_SCALE);
            int x = (int) (pointInWorld.x * FLOAT_TO_INT_SCALE);
            int y = (int) (pointInWorld.y * FLOAT_TO_INT_SCALE);
            int z = (int) (pointInWorld.z * FLOAT_TO_INT_SCALE);

            atomic_add(&variance_data[3 * idx], variance_x);
            atomic_add(&variance_data[3 * idx + 1], variance_y);
            atomic_add(&variance_data[3 * idx + 2], variance_z);
            atomic_add(&centroid_data[3 * idx], x);
            atomic_add(&centroid_data[3 * idx + 1], y);
            atomic_add(&centroid_data[3 * idx + 2], z);
            atomic_inc(&counter_data[idx]);

            // visibility cleanup
 //       }
    }
}

void kernel averageMapImagesKernel(global int* centroid_buffer,
                                   global int* variance_buffer,
                                   global int* counter_buffer,
                                   global float* params,
                                   write_only image2d_t centroid_x,
                                   write_only image2d_t centroid_y,
                                   write_only image2d_t centroid_z,
                                   write_only image2d_t variance_x,
                                   write_only image2d_t variance_y,
                                   write_only image2d_t variance_z,
                                   write_only image2d_t counter)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int2 key = (int2) (idx_x, idx_y);

    int idx = get_idx_in_layer(idx_x, idx_y, params);


    int new_cnt = counter_buffer[idx];

    write_imageui(counter, key, (uint4)(new_cnt,0,0,0));

    if (new_cnt > 0)
    {
        float scalar = (float) FLOAT_TO_INT_SCALE;
        float new_x = ((float) centroid_buffer[3 * idx]) / scalar;
        float new_y = ((float) centroid_buffer[3 * idx + 1]) / scalar;
        float new_z = ((float) centroid_buffer[3 * idx + 2]) / scalar;
        float new_vx = ((float) variance_buffer[3 * idx]) / scalar;
        float new_vy = ((float) variance_buffer[3 * idx + 1]) / scalar;
        float new_vz = ((float) variance_buffer[3 * idx + 2]) / scalar;

        float var_x = (new_vx - new_x * new_x / new_cnt) / ((float) (new_cnt - 1));
        float var_y = (new_vy - new_y * new_y / new_cnt) / ((float) (new_cnt - 1));
        float var_z = (new_vz - new_z * new_z / new_cnt) / ((float) (new_cnt - 1));
        float x = new_x / new_cnt;
        float y = new_y / new_cnt;
        float z = new_z / new_cnt;

        write_imagef(centroid_x, key, (float4)(x,0,0,0));
        write_imagef(centroid_y, key, (float4)(y,0,0,0));
        write_imagef(centroid_z, key, (float4)(z,0,0,0));
        write_imagef(variance_x, key, (float4)(var_x,0,0,0));
        write_imagef(variance_y, key, (float4)(var_y,0,0,0));
        write_imagef(variance_z, key, (float4)(var_z,0,0,0));
    }
}





