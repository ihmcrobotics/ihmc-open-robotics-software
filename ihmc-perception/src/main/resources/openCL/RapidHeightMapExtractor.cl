#define LOCAL_CELL_SIZE 0
#define LOCAL_CENTER_INDEX 1
#define DEPTH_INPUT_HEIGHT 2
#define DEPTH_INPUT_WIDTH 3
#define HEIGHT_MAP_CENTER_X 4
#define HEIGHT_MAP_CENTER_Y 5
#define MODE 6
#define DEPTH_CX 7
#define DEPTH_CY 8
#define DEPTH_FX 9
#define DEPTH_FY 10
#define GLOBAL_CELL_SIZE 11
#define GLOBAL_CENTER_INDEX 12
#define ROBOT_COLLISION_RADIUS 13
#define GRID_OFFSET_X 14
#define HEIGHT_FILTER_ALPHA 15
#define LOCAL_CELLS_PER_AXIS 16
#define GLOBAL_CELLS_PER_AXIS 17
#define HEIGHT_SCALING_FACTOR 18
#define MIN_HEIGHT_REGISTRATION 19
#define MAX_HEIGHT_REGISTRATION 20
#define MIN_HEIGHT_DIFFERENCE 21
#define MAX_HEIGHT_DIFFERENCE 22
#define SEARCH_WINDOW_HEIGHT 23
#define SEARCH_WINDOW_WIDTH 24

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

float compute_footstep_cost(global float* )
{
}

float get_height_on_plane(float x, float y, global float *plane)
{
   float height = (plane[3] - (plane[0] * x + plane[1] * y)) / plane[2];
   return height;
}

void kernel heightMapUpdateKernel(read_write image2d_t in,
                                  read_write image2d_t out,
                                  global float *params,
                                  global float *sensorToZUpFrameTf,
                                  global float *zUpToSensorFrameTf)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   float3 normal;
   float3 centroid;

   float averageHeightZ = 0;
   float3 cellCenterInZUp = (float3) (0.0f, 0.0f, 0.0f);
   cellCenterInZUp.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[LOCAL_CELL_SIZE],
                                               params[LOCAL_CENTER_INDEX]);


   cellCenterInZUp.x += params[GRID_OFFSET_X];

   float halfCellWidth = params[LOCAL_CELL_SIZE] / 2.0f;
   float minX = cellCenterInZUp.x - halfCellWidth;
   float maxX = cellCenterInZUp.x + halfCellWidth;
   float minY = cellCenterInZUp.y - halfCellWidth;
   float maxY = cellCenterInZUp.y + halfCellWidth;

   float3 cellCenterInSensor = transformPoint3D32_2(
      cellCenterInZUp,
      (float3)(zUpToSensorFrameTf[0], zUpToSensorFrameTf[1], zUpToSensorFrameTf[2]),
      (float3)(zUpToSensorFrameTf[4], zUpToSensorFrameTf[5], zUpToSensorFrameTf[6]),
      (float3)(zUpToSensorFrameTf[8], zUpToSensorFrameTf[9], zUpToSensorFrameTf[10]),
      (float3)(zUpToSensorFrameTf[3], zUpToSensorFrameTf[7], zUpToSensorFrameTf[11]));

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
   //       xIndex, yIndex, cellCenterInZUp.x, cellCenterInZUp.y, cellCenterInZUp.z,
   //       cellCenterInSensor.x, cellCenterInSensor.y, cellCenterInSensor.z, projectedPoint.x, projectedPoint.y);

   int count = 0;

   for (int pitch_count_offset = - (int) (params[SEARCH_WINDOW_HEIGHT] / 2); pitch_count_offset <  (int) (params[SEARCH_WINDOW_HEIGHT] / 2 + 1); pitch_count_offset+=3)
   {
      for (int yaw_count_offset = - (int) (params[SEARCH_WINDOW_WIDTH] / 2); yaw_count_offset <  (int) (params[SEARCH_WINDOW_WIDTH] / 2 + 1); yaw_count_offset+=3)
      {
         int yaw_count = projectedPoint.x + yaw_count_offset;
         int pitch_count = projectedPoint.y + pitch_count_offset;

         if ((yaw_count >= 0) && (yaw_count < (int)params[DEPTH_INPUT_WIDTH]) && (pitch_count >= 0) && (pitch_count < (int)params[DEPTH_INPUT_HEIGHT]))
         {
            float depth = ((float)read_imageui(in, (int2) (yaw_count, pitch_count)).x) / (float) 1000;

            float3 queryPointInSensor;
            float3 queryPointInWorld;
            if (params[MODE] == 0) // Spherical
            {
               queryPointInSensor = back_project_spherical(yaw_count, pitch_count, depth, params);
            }
            else if (params[MODE] == 1) // Perspective
            {
               queryPointInSensor = back_project_perspective((int2) (yaw_count, pitch_count), depth, params);
            }

            queryPointInWorld = transformPoint3D32_2(
            queryPointInSensor,
            (float3)(sensorToZUpFrameTf[0], sensorToZUpFrameTf[1], sensorToZUpFrameTf[2]),
            (float3)(sensorToZUpFrameTf[4], sensorToZUpFrameTf[5], sensorToZUpFrameTf[6]),
            (float3)(sensorToZUpFrameTf[8], sensorToZUpFrameTf[9], sensorToZUpFrameTf[10]),
            (float3)(sensorToZUpFrameTf[3], sensorToZUpFrameTf[7], sensorToZUpFrameTf[11]));

            //printf("xIndex: %d, yIndex: %d\tcellCenter: (%f, %f, %f)\tprojectedPoint: (%d, %d)\t(yaw: %d, pitch: %d)\tdepth: %f\tqueryPoint: (%f,%f,%f)\tLimits: (x:[%f,%f], y:[%f,%f])\n",
            //   xIndex, yIndex, cellCenterInZUp.x, cellCenterInZUp.y, cellCenterInZUp.z, projectedPoint.x, projectedPoint.y,
            //   yaw_count, pitch_count, depth, queryPointInWorld.x, queryPointInWorld.y, queryPointInWorld.z, minX, maxX, minY, maxY);


            //printf("xIndex: %d, yIndex: %d \tWorld Point: (%f, %f, %f), Sensor Point (Z-fwd): (%f, %f, %f) -> Image Point: (%d, %d)\n", xIndex, yIndex,
            //      cellCenterInZUp.x, cellCenterInZUp.y, cellCenterInZUp.z,
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

   int cellsPerAxis = (int)params[LOCAL_CELLS_PER_AXIS];

   if (count > 0)
   {
      averageHeightZ = averageHeightZ / (float)(count);
      averageHeightZ = clamp(averageHeightZ, -5.f, 5.0f);

      //printf("xIndex: %d, yIndex: %d, count: %d, averageHeightZ: %f\n", xIndex, yIndex, count, averageHeightZ);
   }
   else
   {
      // this is slightly below the floor height of what we'll accept
      averageHeightZ = params[MIN_HEIGHT_REGISTRATION] - 0.05f;
   }
   write_imageui(out, (int2)(yIndex, xIndex), (uint4)((int)( (averageHeightZ) * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
}

void kernel heightMapRegistrationKernel(read_write image2d_t localMap,
                                        read_write image2d_t globalMap,
                                        global float *params,
                                        global float *worldToZUpFrameTf)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   // Create a 3D point in world frame
   float3 cellCenterInWorld = (float3)(0, 0, 0);
   cellCenterInWorld.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[GLOBAL_CELL_SIZE],
                                               params[GLOBAL_CENTER_INDEX]);

   // Transform the point to the ZUp frame
   float3 cellCenterInZUpFrame = transformPoint3D32_2(
      cellCenterInWorld,
      (float3)(worldToZUpFrameTf[0], worldToZUpFrameTf[1], worldToZUpFrameTf[2]),
      (float3)(worldToZUpFrameTf[4], worldToZUpFrameTf[5], worldToZUpFrameTf[6]),
      (float3)(worldToZUpFrameTf[8], worldToZUpFrameTf[9], worldToZUpFrameTf[10]),
      (float3)(worldToZUpFrameTf[3], worldToZUpFrameTf[7], worldToZUpFrameTf[11]));

   // Check if the point is within the robot's collision radius
   bool isColliding = length(cellCenterInZUpFrame.xy) < params[ROBOT_COLLISION_RADIUS];
   if (isColliding)
   {
       return;
   }

   cellCenterInZUpFrame.x -= params[GRID_OFFSET_X];

   // Compute the local cell index in the local map
   int2 localCellIndex = coordinate_to_indices(
      (float2)(cellCenterInZUpFrame.x, cellCenterInZUpFrame.y),
      (float2)(0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
      params[LOCAL_CELL_SIZE],
      params[LOCAL_CENTER_INDEX]);

   int localCellsPerAxis = (int) params[LOCAL_CELLS_PER_AXIS];

   // Extract the height from the local map at the local cell index (if within bounds)
   float previousHeight = (float) read_imageui(globalMap, (int2)(yIndex, xIndex)).x / params[HEIGHT_SCALING_FACTOR];
   float height = params[MIN_HEIGHT_REGISTRATION] - 0.05f;

   if (localCellIndex.x >= 0 && localCellIndex.x < localCellsPerAxis && localCellIndex.y >= 0 && localCellIndex.y < localCellsPerAxis)
   {
      height = (float)read_imageui(localMap, (int2)(localCellIndex.y, localCellIndex.x)).x / params[HEIGHT_SCALING_FACTOR];
   }

   float finalHeight = previousHeight;

   // Filter the height value if it is within the registration height range and not colliding with the robot
   if (!isColliding && height > params[MIN_HEIGHT_REGISTRATION] && height < params[MAX_HEIGHT_REGISTRATION])
   {
      // Apply a poor man's mahalanobis filter on the data
      float height_diff = height - previousHeight;
      if (height_diff > params[MIN_HEIGHT_DIFFERENCE] && height_diff < params[MAX_HEIGHT_DIFFERENCE])
      {
         finalHeight = previousHeight * params[HEIGHT_FILTER_ALPHA] + height * (1.0f - params[HEIGHT_FILTER_ALPHA]);
      }
      else
      {
         // the difference between the incoming data and the old data was too much, reset it to the incoming data completely
         finalHeight = height;
      }
   }

   // Put the height value in the global map at the global cell index
   write_imageui(globalMap, (int2)(yIndex, xIndex), (uint4)((int)(finalHeight * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
}

void kernel footstepCostKernel(read_write image2d_t heightMap,
                                       read_write image2d_t costMap,
                                       global float *params,
                                       global float *midFootGoalPosition,
                                       global float *leftStartPose,
                                       global float *rightStartPose)
{
   // Extract the indices
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   // Compute the cell center in the world frame
   float3 cellCenterInWorld = (float3)(0, 0, 0);
   cellCenterInWorld.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[GLOBAL_CELL_SIZE],
                                               params[GLOBAL_CENTER_INDEX]);

   // In a 5x5 neighborhood around the current cell compute the steppability as 4 booleans to be stored in the cost map first four bits
   int steppability = 0;
   int m = 2;
   for (int xOffset = -m; xOffset < m+1; xOffset++)
   {
      for (int yOffset = -m; yOffset < m+1; yOffset++)
      {
            compute_footstep_cost()
      }
   }


   // Put the steppability count for each out of 4 layers around the current cell as boolean in first 4 bits of the cost map



}