#define HEIGHT_MAP_RESOLUTION 0
#define HEIGHT_MAP_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define HEIGHT_MAP_CENTER_X 4
#define HEIGHT_MAP_CENTER_Y 5

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

void kernel heightMapUpdateKernel(read_only image2d_t in,
                                  read_write image2d_t out,
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

  int2 unitSphericalCoordinates = spherical_projection(cellCenterInSensor, params); // (x, y) <-> (pitch, yaw)

  int WINDOW_WIDTH = 40;

  float halfCellWidth = params[HEIGHT_MAP_RESOLUTION] / 2.0f;
  float minX = cellCenterInWorld.x - halfCellWidth;
  float maxX = cellCenterInWorld.x + halfCellWidth;
  float minY = cellCenterInWorld.y - halfCellWidth;
  float maxY = cellCenterInWorld.y + halfCellWidth;

  int count = 0;

  for (int pitch_count = 3 / 4 * (int)params[DEPTH_INPUT_HEIGHT]; pitch_count < (int)params[DEPTH_INPUT_HEIGHT]; pitch_count++)
  {
    for (int yaw_count_offset = -WINDOW_WIDTH / 2; yaw_count_offset < WINDOW_WIDTH / 2 + 1; yaw_count_offset++)
    {
      int yaw_count = unitSphericalCoordinates.y + yaw_count_offset;

      if ((yaw_count >= 0) && (yaw_count < (int)params[DEPTH_INPUT_WIDTH]) && (pitch_count >= 0) && (pitch_count < (int)params[DEPTH_INPUT_HEIGHT]))
      {
        float radius = ((float)read_imageui(in, (int2) (yaw_count, pitch_count)).x) / (float)1000;

        float3 testPointInSensorFrame = back_project_spherical(yaw_count, pitch_count, radius, params);
        float3 testPointInWorldFrame = transformPoint3D32_2(
            testPointInSensorFrame,
            (float3)(sensorToWorldTf[0], sensorToWorldTf[1], sensorToWorldTf[2]),
            (float3)(sensorToWorldTf[4], sensorToWorldTf[5], sensorToWorldTf[6]),
            (float3)(sensorToWorldTf[8], sensorToWorldTf[9], sensorToWorldTf[10]),
            (float3)(sensorToWorldTf[3], sensorToWorldTf[7], sensorToWorldTf[11]));


        if (fabs(testPointInWorldFrame.x - cellCenterInWorld.x) < halfCellWidth
            && fabs(testPointInWorldFrame.y - cellCenterInWorld.y) < halfCellWidth)
        {
      //printf("testPointInWorldFrame: %f, %f, %f <-> cellCenterInWorld: %f, %f, %f \n", testPointInWorldFrame.x, testPointInWorldFrame.y, testPointInWorldFrame.z, cellCenterInWorld.x, cellCenterInWorld.y, cellCenterInWorld.z);

          count++;
          averageHeightZ += testPointInWorldFrame.z;
        }
      }
    }
  }



  if (count > 0)
  {
    //averageHeightZ = averageHeightZ / (float)(count) - get_height_on_plane(cellCenterInSensor.x, cellCenterInSensor.y, plane);
    averageHeightZ = averageHeightZ / (float)(count);
    averageHeightZ = clamp(averageHeightZ, -2.0f, 1.5f);

    write_imageui(out, (int2)(xIndex, yIndex), (uint4)((int)((2.0f + averageHeightZ) * 10000), 0, 0, 0));
  }
  else
  {
    write_imageui(out, (int2)(xIndex, yIndex), (uint4)(0, 0, 0, 0));
  }
}
