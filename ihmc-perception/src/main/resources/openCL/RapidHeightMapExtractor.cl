#define HEIGHT_MAP_RESOLUTION 0
#define HEIGHT_MAP_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define HEIGHT_MAP_CENTER_X 4
#define HEIGHT_MAP_CENTER_Y 5
#define MODE 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define DEPTH_FX 9
#define DEPTH_FY 10

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

float3 back_project_perspective(int2 pos, float Z, global float* params)
{
   float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
   float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;

   float3 point = (float3) (Z, -X, -Y);
   return point;
}

int2 perspective_projection(float3 point, global float* params)
{
   float x = point.x / point.z * params[DEPTH_FX] + params[DEPTH_CX];
   float y = point.y / point.z * params[DEPTH_FY] + params[DEPTH_CY];

   return (int2) (x, y);
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
                                  write_only image2d_t out,
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
   float3 cellCenterInWorld = (float3) (0.0f, 0.0f, 0.65f);
   cellCenterInWorld.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[HEIGHT_MAP_RESOLUTION],
                                               params[HEIGHT_MAP_CENTER_INDEX]);


   cellCenterInWorld.x += 1.5f;

   int WINDOW_WIDTH = 100;
   int WINDOW_HEIGHT = 180;

   float halfCellWidth = params[HEIGHT_MAP_RESOLUTION] / 4.0f;
   float minX = cellCenterInWorld.x - halfCellWidth;
   float maxX = cellCenterInWorld.x + halfCellWidth;
   float minY = cellCenterInWorld.y - halfCellWidth;
   float maxY = cellCenterInWorld.y + halfCellWidth;

   float3 cellCenterInSensor = transformPoint3D32_2(
      cellCenterInWorld,
      (float3)(worldToSensorTf[0], worldToSensorTf[1], worldToSensorTf[2]),
      (float3)(worldToSensorTf[4], worldToSensorTf[5], worldToSensorTf[6]),
      (float3)(worldToSensorTf[8], worldToSensorTf[9], worldToSensorTf[10]),
      (float3)(worldToSensorTf[3], worldToSensorTf[7], worldToSensorTf[11]));

   int2 projectedPoint;
   if (params[MODE] == 0) // Spherical Projection
   {
      projectedPoint = spherical_projection(cellCenterInSensor, params);
   }
   else if (params[MODE] == 1) // Perspective Projection
   {
      // convert cellCenterInSensor to z-forward, x-right, y-down
      float3 cellCenterInSensorZfwd = (float3) (-cellCenterInSensor.y, -cellCenterInSensor.z, cellCenterInSensor.x);

      if (cellCenterInSensorZfwd.z < 0)
        return;

      projectedPoint = perspective_projection(cellCenterInSensorZfwd, params);
   }

   //printf("xIndex: %d, yIndex: %d\tcellCenterWorld: (%f, %f, %f)\tcellCenter: (%f, %f, %f)\tprojectedPoint: (%d, %d)\n",
   //       xIndex, yIndex, cellCenterInWorld.x, cellCenterInWorld.y, cellCenterInWorld.z,
   //       cellCenterInSensor.x, cellCenterInSensor.y, cellCenterInSensor.z, projectedPoint.x, projectedPoint.y);

   int count = 0;

   for (int pitch_count_offset = -WINDOW_HEIGHT / 2; pitch_count_offset < WINDOW_HEIGHT / 2 + 1; pitch_count_offset+=3)
   {
      for (int yaw_count_offset = -WINDOW_WIDTH / 2; yaw_count_offset < WINDOW_WIDTH / 2 + 1; yaw_count_offset+=3)
      {
         int yaw_count = projectedPoint.x + yaw_count_offset;
         int pitch_count = projectedPoint.y + pitch_count_offset;

         if ((yaw_count >= 0) && (yaw_count < (int)params[DEPTH_INPUT_WIDTH]) && (pitch_count >= 0) && (pitch_count < (int)params[DEPTH_INPUT_HEIGHT]))
         {
            float radius = ((float)read_imageui(in, (int2) (yaw_count, pitch_count)).x) / (float) 1000;

            float3 queryPointInSensor;
            float3 queryPointInWorld;
            if (params[MODE] == 0) // Spherical
            {
               queryPointInSensor = back_project_spherical(yaw_count,pitch_count,radius,params);
            }
            else if (params[MODE] == 1) // Perspective
            {
               queryPointInSensor = back_project_perspective((int2) (yaw_count, pitch_count), radius, params);
            }

            queryPointInWorld = transformPoint3D32_2(
            queryPointInSensor,
            (float3)(sensorToWorldTf[0], sensorToWorldTf[1], sensorToWorldTf[2]),
            (float3)(sensorToWorldTf[4], sensorToWorldTf[5], sensorToWorldTf[6]),
            (float3)(sensorToWorldTf[8], sensorToWorldTf[9], sensorToWorldTf[10]),
            (float3)(sensorToWorldTf[3], sensorToWorldTf[7], sensorToWorldTf[11]));

            //printf("xIndex: %d, yIndex: %d\tcellCenter: (%f, %f, %f)\tprojectedPoint: (%d, %d)\t(yaw: %d, pitch: %d)\tdepth: %f\tqueryPoint: (%f,%f,%f)\tLimits: (x:[%f,%f], y:[%f,%f])\n",
            //   xIndex, yIndex, cellCenterInWorld.x, cellCenterInWorld.y, cellCenterInWorld.z, projectedPoint.x, projectedPoint.y,
            //   yaw_count, pitch_count, radius, queryPointInWorld.x, queryPointInWorld.y, queryPointInWorld.z, minX, maxX, minY, maxY);


            //printf("xIndex: %d, yIndex: %d \tWorld Point: (%f, %f, %f), Sensor Point (Z-fwd): (%f, %f, %f) -> Image Point: (%d, %d)\n", xIndex, yIndex,
            //      cellCenterInWorld.x, cellCenterInWorld.y, cellCenterInWorld.z,
            //      cellCenterInSensor.x, cellCenterInSensor.y, cellCenterInSensor.z,
            //      projectedPoint.x, projectedPoint.y);

            if (queryPointInWorld.x > minX && queryPointInWorld.x < maxX && queryPointInWorld.y > minY && queryPointInWorld.y < maxY)
            {
               //printf("HIT......: (%f, %f, %f), minX: %f, maxX: %f, minY: %f, maxY: %f\n", queryPointInWorld.x, queryPointInWorld.y, queryPointInWorld.z, minX, maxX, minY, maxY);
               count++;
               averageHeightZ += queryPointInWorld.z;
            }
         }
      }
   }

   int cellsPerAxis = 2 * params[HEIGHT_MAP_CENTER_INDEX] + 1;

   if (count > 0)
   {
      averageHeightZ = averageHeightZ / (float)(count);
      averageHeightZ = clamp(averageHeightZ, -5.f, 5.0f);

      write_imageui(out, (int2)(xIndex, yIndex), (uint4)((int)( (averageHeightZ - 0.65f) * 10000.0f), 0, 0, 0));

      //printf("xIndex: %d, yIndex: %d, count: %d, averageHeightZ: %f\n", xIndex, yIndex, count, averageHeightZ);
   }
   else
   {
      write_imageui(out, (int2)(xIndex, yIndex), (uint4)(0, 0, 0, 0));
   }
}
