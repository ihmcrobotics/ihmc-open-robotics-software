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

#define RESOLUTION 0
#define MIN_VALID_DISTANCE 1
#define MAX_HEIGHT_RANGE 2
#define RAMPED_HEIGHT_RANGE_A 3
#define RAMPED_HEIGHT_RANGE_B 4
#define RAMPED_HEIGHT_RANGE_C 5
#define CENTER_INDEX 6
#define CELLS_OVER_FOR_NORMAL 7

#define DEPTH_CX 0
#define DEPTH_CY 1
#define DEPTH_FX 2
#define DEPTH_FY 3

int clamp_val(int x, int min_x, int max_x) { return max(min(x, max_x), min_x); }

float rotate_point(float3 point, float3 r) { return dot(point, r); }

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

/*
 * Resolution is the size of a "pixel" in the height map.
 * Center index is the index from the edge of the height map region.
 * Center is the distance in meters from world to the center of the height map region.
 * Coordinate is the distance in meters from world to the "pixel" in the height map.
 * Therefore, we get the index of the "pixel" (x, y, bottom left corner) from a world point.
 */
int coordinate_to_index(float coordinate, float center, float resolution, int centerIndex)
{
   int index = ((int) round((coordinate - center) / resolution)) + centerIndex;
   return clamp_val(index, 0, 2 * centerIndex + 1);
}

int get_x_idx(float x, float center, global float* params) { return coordinate_to_index(x, center, params[RESOLUTION], params[CENTER_INDEX]); }

int get_y_idx(float y, float center, global float* params) { return coordinate_to_index(y, center, params[RESOLUTION], params[CENTER_INDEX]); }

int indices_to_key(int x_index, int y_index, int centerIndex) { return x_index + y_index * (2 * centerIndex + 1); }

int get_idx_in_layer(int idx_x, int idx_y, global float* params) { return indices_to_key(idx_x, idx_y, params[CENTER_INDEX]); }

bool is_valid(float3 point, float3 sensor, global float* params)
{
   float d = point_sensor_distance(point, sensor);
   float min = 0.0;

   if (d < params[MIN_VALID_DISTANCE] * params[MIN_VALID_DISTANCE])
   {
      printf("Returning that a point is not valid\n");
      return false;
   }

   return true;
   // float dxy = max(sqrt(x * x + y * y) - params[RAMPED_HEIGHT_RANGE_B], min);

   // if (z - sz > dxy * params[RAMPED_HEIGHT_RANGE_A] + params[RAMPED_HEIGHT_RANGE_C] || z - sz > params[MAX_HEIGHT_RANGE])
   //{
   // printf("Not valid from ramp");
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
   return (float3) (point_in_image_frame.z, -point_in_image_frame.x, -point_in_image_frame.y);
}

float3 get_point_from_image(read_only image2d_t depth_image, int x, int y, global float* params)
{
   int2 key = (int2) (x, y);
   //    float Z = ((float)read_imageui(depth_image, key).x)/(float) 1000;
   float Z = ((float) read_imagef(depth_image, key).x);

   //    printf("Z: %.3f\n", Z);

   if (Z > 0.1f)
   {
      return project_from_image_frame_to_camera_frame(back_project_to_image_frame(key, Z, params));
   }
   else
   {
      return (float3) (0.0, 0.0, 0.0);
   }
}

void kernel zeroValuesKernel(global float* params, global int* centroid_data, global int* variance_data, global int* counter_data)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);

   int idx = get_idx_in_layer(idx_x, idx_y, params);

   centroid_data[3 * idx] = 0;
   centroid_data[3 * idx + 1] = 0;
   centroid_data[3 * idx + 2] = 0;
   variance_data[idx] = 0;
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
   int image_x = get_global_id(0);
   int image_y = get_global_id(1);

   float3 point_in_camera_frame = get_point_from_image(depth_image, image_x, image_y, intrinsics);

   //    printf("PointInCameraFrame(%.3lf,%.3lf,%.3lf), \n", point_in_camera_frame.x, point_in_camera_frame.y, point_in_camera_frame.z);

   float3 sensor = (float3) (localization[tx], localization[ty], localization[tz]);
   float3 rx = (float3) (localization[r00], localization[r01], localization[r02]);
   float3 ry = (float3) (localization[r10], localization[r11], localization[r12]);
   float3 rz = (float3) (localization[r20], localization[r21], localization[r22]);

   float3 pointInWorld = transform_point(point_in_camera_frame, rx, ry, rz, sensor);

   if (is_valid(pointInWorld, sensor, params))
   {
      int idx_x = get_x_idx(pointInWorld.x, localization[centerX], params);
      int idx_y = get_y_idx(pointInWorld.y, localization[centerY], params);
      // row major index
      int idx = get_idx_in_layer(idx_x, idx_y, params);

      // TODO pretty sure this is always true by construction
      float new_variance_z = pointInWorld.z * pointInWorld.z;

      int variance_z = (int) (new_variance_z * FLOAT_TO_INT_SCALE);
      int x = (int) (pointInWorld.x * FLOAT_TO_INT_SCALE);
      int y = (int) (pointInWorld.y * FLOAT_TO_INT_SCALE);
      int z = (int) (pointInWorld.z * FLOAT_TO_INT_SCALE);

      atomic_add(&centroid_data[3 * idx], x);
      atomic_add(&centroid_data[3 * idx + 1], y);
      atomic_add(&centroid_data[3 * idx + 2], z);
      atomic_add(&variance_data[idx], variance_z);
      atomic_inc(&counter_data[idx]);

      // visibility cleanup
   }
}

void kernel averageMapImagesKernel(global int* centroid_buffer,
                                   global int* variance_buffer,
                                   global int* counter_buffer,
                                   global float* params,
                                   write_only image2d_t centroid_x,
                                   write_only image2d_t centroid_y,
                                   write_only image2d_t centroid_z,
                                   write_only image2d_t variance_z,
                                   write_only image2d_t counter)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);

   int2 key = (int2) (idx_x, idx_y);

   int idx = get_idx_in_layer(idx_x, idx_y, params);

   int new_cnt = counter_buffer[idx];

   write_imageui(counter, key, (uint4) (new_cnt, 0, 0, 0));

   if (new_cnt > 0)
   {
      float scalar = (float) FLOAT_TO_INT_SCALE;
      float new_x = ((float) centroid_buffer[3 * idx]) / scalar;
      float new_y = ((float) centroid_buffer[3 * idx + 1]) / scalar;
      float new_z = ((float) centroid_buffer[3 * idx + 2]) / scalar;
      float new_vz = ((float) variance_buffer[idx]) / scalar;

      float var_z = (new_vz - new_z * new_z / new_cnt) / ((float) (new_cnt - 1));
      float x = new_x / new_cnt;
      float y = new_y / new_cnt;
      float z = new_z / new_cnt;

      write_imagef(centroid_x, key, (float4) (x, 0, 0, 0));
      write_imagef(centroid_y, key, (float4) (y, 0, 0, 0));
      write_imagef(centroid_z, key, (float4) (z, 0, 0, 0));
      write_imagef(variance_z, key, (float4) (var_z, 0, 0, 0));
   }
}

void kernel computeNormalsKernel(read_only image2d_t centroid_x,
                                 read_only image2d_t centroid_y,
                                 read_only image2d_t centroid_z,
                                 read_only image2d_t counter,
                                 global float* params,
                                 write_only image2d_t normal_x_mat,
                                 write_only image2d_t normal_y_mat,
                                 write_only image2d_t normal_z_mat)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);

   int2 key = (int2) (idx_x, idx_y);

   int cnt = read_imageui(counter, key).x;

   if (cnt > 0)
   {
      float normal_x = 0;
      float normal_y = 0;
      int count_x = 0;
      int count_y = 0;
      float x = read_imagef(centroid_x, key).x;
      float y = read_imagef(centroid_y, key).x;
      float z = read_imagef(centroid_z, key).x;
      int max_val = 2 * params[CENTER_INDEX] + 1;

      for (int i = -params[CELLS_OVER_FOR_NORMAL]; i <= params[CELLS_OVER_FOR_NORMAL]; i++)
      {
         if (i == 0)
            continue;

         int mod_x = idx_x + i;
         int mod_y = idx_y + i;
         int2 key_x = (int2) (mod_x, idx_y);
         int2 key_y = (int2) (idx_x, mod_y);

         if (mod_x > 0 && mod_x <= max_val && read_imageui(counter, key_x).x > 0)
         {
            float dzdx = read_imagef(centroid_z, key_x).x - z;
            float dx = read_imagef(centroid_x, key_x).x - x;
            normal_x += (-dzdx / dx);
            count_x++;
         }
         if (mod_y > 0 && mod_y <= max_val && read_imageui(counter, key_y).x > 0)
         {
            float dzdy = read_imagef(centroid_z, key_y).x - z;
            float dy = read_imagef(centroid_y, key_y).x - y;
            normal_y += (-dzdy / dy);
            count_y++;
         }
      }

      normal_x /= (float) count_x;
      normal_y /= (float) count_y;
      float norm = sqrt((normal_x * normal_x) + (normal_y * normal_y) + 1);

      normal_x /= norm;
      normal_y /= norm;
      float normal_z = 1.0 / norm;

      // switch the x and y, because the indices are wrong for the global coordinates
      write_imagef(normal_x_mat, key, (float4) (normal_x, 0, 0, 0));
      write_imagef(normal_y_mat, key, (float4) (normal_y, 0, 0, 0));
      write_imagef(normal_z_mat, key, (float4) (normal_z, 0, 0, 0));
   }
}
