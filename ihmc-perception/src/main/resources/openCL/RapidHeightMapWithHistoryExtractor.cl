#define HEIGHT_MAP_RESOLUTION 0
#define HEIGHT_MAP_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define HEIGHT_MAP_CENTER_X 4
#define HEIGHT_MAP_CENTER_Y 5

#define BUFFER_LENGTH 6

#define VERTICAL_FOV M_PI_2_F
#define HORIZONTAL_FOV (2.0f * M_PI_F)

/**
 * Converts a coordinate (yaw, pitch) in the lidar sensor to a point in the sensor frame
 **/
float3 back_project_spherical(int yaw_index, int pitch_index, float depth, global float *params)
{
  int yawCountsFromCenter = -yaw_index - (params[DEPTH_INPUT_WIDTH] / 2);
  int pitchCountsFromCenter = -(pitch_index - (params[DEPTH_INPUT_HEIGHT] / 2));

  float yaw = yawCountsFromCenter / (float)params[DEPTH_INPUT_WIDTH] * HORIZONTAL_FOV;
  float pitch = pitchCountsFromCenter / (float)params[DEPTH_INPUT_HEIGHT] * VERTICAL_FOV;

  float r = depth * cos(pitch);

  float px = r * cos(yaw);
  float py = r * sin(yaw);
  float pz = depth * sin(pitch);

  return (float3)(px, py, pz);
}

/**
 * Converts a point in the sensor frame into a coordinate in the lidar sensor.
 * coordinates returned are (int2) (pitch, yaw)
 **/
int2 spherical_projection(float3 cellCenter, global float *params)
{
  float pitchUnit = VERTICAL_FOV / (params[DEPTH_INPUT_HEIGHT]);
  float yawUnit = HORIZONTAL_FOV / (params[DEPTH_INPUT_WIDTH]);

  int pitchOffset = params[DEPTH_INPUT_HEIGHT] / 2;
  int yawOffset = params[DEPTH_INPUT_WIDTH] / 2;

  float x = cellCenter.x;
  float y = cellCenter.y;
  float z = cellCenter.z;

  float radius = length(cellCenter.xy);

  float pitch = atan2(z, radius);
  int pitchCount = (pitchOffset) - (int)(pitch / pitchUnit);

  float yaw = atan2(-y, x);
  int yawCount = (yawOffset) + (int)(yaw / yawUnit);

  return (int2) (pitchCount, yawCount);
}

float get_height_on_plane(float x, float y, global float *plane)
{
  float height = (plane[3] - (plane[0] * x + plane[1] * y)) / plane[2];
  return height;
}

void initializeCellData(int data_key,
                        global float *height_samples,
                        global float *variance_samples,
                        global int *samples_per_buffered_value,
                        global int *buffer_write_keys,
                        global int *entries_in_buffer)
{
  height_samples[data_key] = -2.0f;
  variance_samples[data_key] = 0.0f;
  samples_per_buffered_value[data_key] = 1;
  buffer_write_keys[data_key] = 0;
  entries_in_buffer[data_key] = 0;
}

void kernel initializeDataStructureKernel(global float *params,
                                          write_only image2d_t data_keys,
                                          global float *height_samples,
                                          global float *variance_samples,
                                          global int *samples_per_buffered_value,
                                          global int *buffer_write_keys,
                                          global int *entries_in_buffer)
{
  int xIndex = get_global_id(0);
  int yIndex = get_global_id(1);

  int2 indices = (int2)(xIndex, yIndex);

  int data_key = indices_to_key(xIndex, yIndex, params[HEIGHT_MAP_CENTER_INDEX]);

  write_imageui(data_keys, indices, (uint4)(0, 0, 0, 0));

  initializeCellData(data_key, height_samples, variance_samples, samples_per_buffered_value, buffer_write_keys, entries_in_buffer);
}

void kernel translateHeightMapKernel(global float *params,
                                     read_only image2d_t old_data_keys,
                                     write_only image2d_t new_data_keys,
                                     global float *old_origin,
                                     global float *height_samples,
                                     global float *variance_samples,
                                     global int *samples_per_buffered_value,
                                     global int *buffer_write_keys,
                                     global int *entries_in_buffer)
{
  int xIndex = get_global_id(0);
  int yIndex = get_global_id(1);

  int2 indices = (int2) (xIndex, yIndex);

  int center_index = (int) params[HEIGHT_MAP_CENTER_INDEX];
  float2 center = (float2) (params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]);
  float resolution = params[HEIGHT_MAP_RESOLUTION];

  int center_x_index = key_to_x_index(center_index, center_index);
  int center_y_index = key_to_y_index(center_index, center_index);

  int old_center_x_index = coordinate_to_index(old_origin[0], center.x, resolution, center_index);
  int old_center_y_index = coordinate_to_index(old_origin[1], center.y, resolution, center_index);

  int x_translation = center_x_index - old_center_x_index;
  int y_translation = center_y_index - old_center_y_index;

  int cells_per_side = 2 * center_index + 1;
  int old_x_index = xIndex - x_translation;
  int old_y_index = yIndex - y_translation;

  bool old_in_bounds = old_x_index >= 0 && old_x_index < cells_per_side && old_y_index >= 0 && old_y_index < cells_per_side;

  int data_buffer_key;
  if (old_in_bounds)
  {
    int2 old_indices = (int2) (old_x_index, old_y_index);
    data_buffer_key = read_imageui(old_data_keys, old_indices).x;
  }
  else
  {
    while (old_x_index < 0)
      old_x_index += cells_per_side;
    while (old_x_index >= cells_per_side)
      old_x_index -= cells_per_side;
    while (old_y_index < 0)
      old_y_index += cells_per_side;
    while (old_y_index >= cells_per_side)
      old_y_index -= cells_per_side;

    int2 old_indices = (int2) (old_x_index, old_y_index);
    data_buffer_key = read_imageui(old_data_keys, old_indices).x;

    initializeCellData(data_buffer_key, height_samples, variance_samples, samples_per_buffered_value, buffer_write_keys, entries_in_buffer);
  }

  write_imageui(new_data_keys, indices, (uint4)(data_buffer_key, 0, 0, 0));
  // TODO add variance to everything, since we just translated.
}

void kernel heightMapUpdateKernel(read_only image2d_t depth_map_in,
                                  read_write image2d_t average_height_out,
                                  global float *params,
                                  global float *sensorToWorldTf,
                                  global float *worldToSensorTf,
                                  global float *plane)
{
  int xIndex = get_global_id(0);
  int yIndex = get_global_id(1);

  float3 normal;
  float3 centroid;

  float averageHeightZ = 0;
  float3 cellCenterInWorld = (float3) (0.0f, 0.0f, -2.0f);;
  cellCenterInWorld.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]),
                                               params[HEIGHT_MAP_RESOLUTION],
                                               params[HEIGHT_MAP_CENTER_INDEX]);

  float3 cellCenterInSensor = transformPoint3D32_2(
      cellCenterInWorld,
      (float3)(worldToSensorTf[0], worldToSensorTf[1], worldToSensorTf[2]),
      (float3)(worldToSensorTf[4], worldToSensorTf[5], worldToSensorTf[6]),
      (float3)(worldToSensorTf[8], worldToSensorTf[9], worldToSensorTf[10]),
      (float3)(worldToSensorTf[3], worldToSensorTf[7], worldToSensorTf[11]));

  int2 projectedPoint = spherical_projection(cellCenterInSensor, params);

  int WINDOW_WIDTH = 20;

  float halfCellWidth = params[HEIGHT_MAP_RESOLUTION] / 2.0f;
  float minX = cellCenterInSensor.x - halfCellWidth;
  float maxX = cellCenterInSensor.x + halfCellWidth;
  float minY = cellCenterInSensor.y - halfCellWidth;
  float maxY = cellCenterInSensor.y + halfCellWidth;

  int count = 0;

  for (int pitch_count = 0; pitch_count < (int)params[DEPTH_INPUT_HEIGHT]; pitch_count++)
  {
    for (int yaw_count_offset = -WINDOW_WIDTH / 2; yaw_count_offset < WINDOW_WIDTH / 2 + 1; yaw_count_offset++)
    {
      int yaw_count = projectedPoint.y + yaw_count_offset;

      if ((yaw_count >= 0) && (yaw_count < (int)params[DEPTH_INPUT_WIDTH]) && (pitch_count >= 0) && (pitch_count < (int)params[DEPTH_INPUT_HEIGHT]))
      {
        float radius = ((float)read_imageui(depth_map_in, (int2) (yaw_count, pitch_count)).x) / (float)1000;

        float3 piontInWorld = back_project_spherical(yaw_count, pitch_count, radius, params);

        if (piontInWorld.x > minX && piontInWorld.x < maxX && piontInWorld.y > minY && piontInWorld.y < maxY)
        {
          count++;
          averageHeightZ += piontInWorld.z;
        }
      }
    }
  }

  if (count > 0)
  {
    averageHeightZ = averageHeightZ / (float)(count) - get_height_on_plane(cellCenterInSensor.x, cellCenterInSensor.y, plane);
    averageHeightZ = clamp(averageHeightZ, -20.f, 1.5f);

    write_imageui(average_height_out, (int2)(xIndex, yIndex), (uint4)((int)((2.0f + averageHeightZ) * 10000.0f), 0, 0, 0));
  }
  else
  {
    write_imageui(average_height_out, (int2)(xIndex, yIndex), (uint4)(0, 0, 0, 0));
  }
}

void kernel computeHeightMapOutputValuesKernel(global float *params,
                                               read_only image2d_t data_keys,
                                               global float *height_samples,
                                               global float *variance_samples,
                                               global int *samples_per_buffered_value,
                                               global int *entries_in_buffer,
                                               write_only image2d_t height_value_out,
                                               write_only image2d_t variance_out)
{
  int xIndex = get_global_id(0);
  int yIndex = get_global_id(1);

  int2 indices = (int2) (xIndex, yIndex);

  int data_key = read_imageui(data_keys, indices).x;
  int entries_to_read = entries_in_buffer[data_key];

  float total_height;
  float total_variance;

  int buffer_length = (int) params[BUFFER_LENGTH];

  if (entries_to_read > 0)
  {
    int entry_to_read = 0;
    int start = data_key * buffer_length;
    int total_samples = samples_per_buffered_value[start];
    total_height = height_samples[start];
    total_variance = variance_samples[start];
    entry_to_read++;
    for (; entry_to_read < entries_to_read; entry_to_read++)
    {
      float height_sample = height_samples[start + entry_to_read];
      float variance_sample = variance_samples[start + entry_to_read];
      int samples = samples_per_buffered_value[start + entry_to_read];

      // TODO double check this math, since it's probably wrong
      total_height += (total_height * total_samples * variance_sample + height_sample * samples * total_variance) / (total_samples * total_variance + samples * variance_sample);
      total_variance += total_variance * total_samples * variance_sample * samples / (total_samples * total_variance + samples * variance_sample);
      total_samples += samples;
    }
    total_height /= total_samples;
    total_variance /= total_samples;
  }
  else
  {
    total_height = -2.0f;
    total_variance = 1.0f;
  }

  total_height = clamp(total_height, -20.f, 1.5f);

  write_imageui(height_value_out, indices, (uint4)((int)((2.0f + total_height) * 10000.0f), 0, 0, 0));
  write_imageui(variance_out, indices, (uint4)((int)((total_variance) * 10000.0f), 0, 0, 0));
}
