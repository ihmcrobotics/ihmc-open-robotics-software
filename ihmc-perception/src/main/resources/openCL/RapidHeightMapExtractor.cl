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
#define CROPPED_WINDOW_CENTER_INDEX 25
#define MIN_CLAMP_HEIGHT 26
#define MAX_CLAMP_HEIGHT 27
#define HEIGHT_OFFSET 28
#define STEPPING_COSINE_THRESHOLD 29
#define STEPPING_CONTACT_THRESHOLD 30
#define CONTACT_WINDOW_SIZE 31
#define SPATIAL_ALPHA 32
#define SEARCH_SKIP_SIZE 33
#define VERTICAL_SEARCH_SIZE 34
#define VERTICAL_SEARCH_RESOLUTION 35
#define FAST_SEARCH_SIZE 36
#define GRAPH_CELLS_PER_AXIS 37
#define GRAPH_PATCH_SIZE 38

#define VERTICAL_FOV M_PI_2_F
#define HORIZONTAL_FOV (2.0f * M_PI_F)

#define SNAP_HEIGHT_MAP_CENTER_X 0
#define SNAP_HEIGHT_MAP_CENTER_Y 1
#define SNAP_GLOBAL_CELL_SIZE 2
#define SNAP_GLOBAL_CENTER_INDEX 3
#define SNAP_CROPPED_WINDOW_CENTER_INDEX 4
#define SNAP_HEIGHT_SCALING_FACTOR 5
#define SNAP_HEIGHT_OFFSET 6
#define SNAP_FOOT_LENGTH 7
#define SNAP_FOOT_WIDTH 8
#define MIN_DISTANCE_FROM_CLIFF_TOPS 9
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 10
#define CLIFF_START_HEIGHT_TO_AVOID 11
#define CLIFF_END_HEIGHT_TO_AVOID 12
#define MIN_SUPPORT_AREA_FRACTION 13
#define MIN_SNAP_HEIGHT_THRESHOLD 14
#define SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE 15
#define INEQUALITY_ACTIVATION_SLOPE 16

#define SNAP_FAILED 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define NOT_ENOUGH_AREA 0
#define VALID 4

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

float get_spatial_average(int xIndex, int yIndex, read_write image2d_t globalMap, global float *params)
{
   // perform a smoothing over neighboring cells
   float heightSum = 0.0f;
   int count = 0;
   for (int i = -1; i < 2; i++)
   {
      for (int j = -1; j < 2; j++)
      {
         if (xIndex + i >= 0 && xIndex + i < params[GLOBAL_CELLS_PER_AXIS] && yIndex + j >= 0 && yIndex + j < params[GLOBAL_CELLS_PER_AXIS])
         {
            heightSum += read_imageui(globalMap, (int2) (yIndex + j, xIndex + i)).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
            count++;
         }
      }
   }
   float heightAverage = heightSum / (float)count;
   return heightAverage;
}

float get_spatial_stddev(int xIndex, int yIndex, float average, read_write image2d_t globalMap, global float *params)
{
   float totalDeviation = 0.0f;
   int count = 0;
   for (int i = -1; i < 2; i++)
   {
      for (int j = -1; j < 2; j++)
      {
         if (xIndex + i >= 0 && xIndex + i < params[GLOBAL_CELLS_PER_AXIS] && yIndex + j >= 0 && yIndex + j < params[GLOBAL_CELLS_PER_AXIS])
         {
            float height = read_imageui(globalMap, (int2) (yIndex + j, xIndex + i)).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
            totalDeviation += (height - average) * (height - average);
            count++;
         }
      }
   }
   float heightStddev = sqrt(totalDeviation / (float)count);
   return heightStddev;
}

float get_spatial_filtered_height(int xIndex, int yIndex, float height, read_write image2d_t globalMap, global float *params)
{
   float averageHeightZ = get_spatial_average(xIndex, yIndex, globalMap, params);
   float heightStddev = get_spatial_stddev(xIndex, yIndex, averageHeightZ, globalMap, params);
   float finalHeight = height;

   if (fabs(finalHeight - averageHeightZ) < 0.5f * heightStddev)
   {
//      finalHeight = averageHeightZ * params[SPATIAL_ALPHA] + finalHeight * (1.0f - params[SPATIAL_ALPHA]);
   }
   else
   {
      finalHeight = averageHeightZ * params[SPATIAL_ALPHA] * 0.0001f + finalHeight * (1.0f - params[SPATIAL_ALPHA] * 0.0001f);
   }

   return finalHeight;
}

bool isTraversible(read_write image2d_t contactMap, read_write image2d_t heightMap,int2 aIndex, int2 bIndex, global float *params)
{
   float c_a = read_imageui(contactMap, aIndex).x;
   float h_a = read_imageui(heightMap, aIndex).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

   float c_b = read_imageui(contactMap, bIndex).x;
   float h_b = read_imageui(heightMap, bIndex).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

   float c_diff = fabs(c_a - c_b);
   float h_diff = fabs(h_a - h_b);

   if (c_diff < 1.0f && h_diff < 0.1f)
   {
      return true;
   }
   return false;
}

void kernel heightMapUpdateKernel(read_write image2d_t in,
                                  read_write image2d_t out,
                                  global float *params,
                                  global float *sensorToZUpFrameTf,
                                  global float *zUpToSensorFrameTf)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   float currentAverageHeight = 0.0f;
   float averageHeightZ = 0.0f;
   float3 cellCenterInZUp = (float3) (0.0f, 0.0f, 0.5f);
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

   int count = 0;
   int skip = (int) params[SEARCH_SKIP_SIZE];

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

   for (int pitch_count_offset = - ((int) params[SEARCH_WINDOW_HEIGHT] / 2); pitch_count_offset < ((int) params[SEARCH_WINDOW_HEIGHT] / 2 + 1); pitch_count_offset+=skip)
   {
      int pitch_count = projectedPoint.y + pitch_count_offset;
      for (int yaw_count_offset = - ((int) params[SEARCH_WINDOW_WIDTH] / 2); yaw_count_offset <  ((int) params[SEARCH_WINDOW_WIDTH] / 2) + 1; yaw_count_offset+=skip)
      {
         int yaw_count = projectedPoint.x + yaw_count_offset;
         if ((yaw_count >= 0) && (yaw_count < (int)params[DEPTH_INPUT_WIDTH]) && (pitch_count >= 0) && (pitch_count < (int)params[DEPTH_INPUT_HEIGHT]))
         {
            float depth = ((float)read_imageui(in, (int2) (yaw_count, pitch_count)).x) / (float) 1000;
            float3 queryPointInSensor;
            if (params[MODE] == 0) // Spherical
            {
               queryPointInSensor = back_project_spherical(yaw_count, pitch_count, depth, params);
            }
            else if (params[MODE] == 1) // Perspective
            {
               queryPointInSensor = back_project_perspective((int2) (yaw_count, pitch_count), depth, params);
            }

            float3 queryPointInZUp = transformPoint3D32_2(
               queryPointInSensor,
               (float3)(sensorToZUpFrameTf[0], sensorToZUpFrameTf[1], sensorToZUpFrameTf[2]),
               (float3)(sensorToZUpFrameTf[4], sensorToZUpFrameTf[5], sensorToZUpFrameTf[6]),
               (float3)(sensorToZUpFrameTf[8], sensorToZUpFrameTf[9], sensorToZUpFrameTf[10]),
               (float3)(sensorToZUpFrameTf[3], sensorToZUpFrameTf[7], sensorToZUpFrameTf[11]));

            if (queryPointInZUp.x > minX && queryPointInZUp.x < maxX && queryPointInZUp.y > minY && queryPointInZUp.y < maxY)
            {
               // remove outliers before averaging for a single cell
               if (count > 1)
               {
                  currentAverageHeight = averageHeightZ / (float)(count);
                  if (fabs(queryPointInZUp.z - currentAverageHeight) > 0.1)
                  {
                     continue;
                  }
               }
               count++;
               averageHeightZ += queryPointInZUp.z;
            }
         }
      }
   }

   if (count > 0)
   {
      averageHeightZ = averageHeightZ / (float)(count);
   }
   else
   {
      // this is slightly below the floor height of what we'll accept
      averageHeightZ = -params[HEIGHT_OFFSET];
   }
   averageHeightZ = clamp(averageHeightZ, params[MIN_CLAMP_HEIGHT], params[MAX_CLAMP_HEIGHT]);
   averageHeightZ += params[HEIGHT_OFFSET];

   write_imageui(out, (int2)(yIndex, xIndex), (uint4)((int)( (averageHeightZ) * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
}

void kernel heightMapRegistrationKernel(read_write image2d_t localMap,
                                        read_write image2d_t globalMap,
                                        read_write image2d_t globalVariance,
                                        global float *params,
                                        global float *worldToZUpFrameTf,
                                        global float *sensorToGroundTf)
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
   float sensorHeight = sensorToGroundTf[11] - 1.5f;
   float previousHeight = (float) read_imageui(globalMap, (int2)(yIndex, xIndex)).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
   float localHeight = previousHeight;

   if (localCellIndex.x >= 0 && localCellIndex.x < localCellsPerAxis && localCellIndex.y >= 0 && localCellIndex.y < localCellsPerAxis)
   {
      localHeight = (float)read_imageui(localMap, (int2)(localCellIndex.y, localCellIndex.x)).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];
   }

   float finalHeight = previousHeight;

   // Filter the height value if it is within the registration height range and not colliding with the robot
   if (!isColliding && (localHeight - sensorHeight) > params[MIN_HEIGHT_REGISTRATION] && (localHeight - sensorHeight) < params[MAX_HEIGHT_REGISTRATION])
   {
      // Apply a poor man's mahalanobis filter on the data
      float height_diff = fabs(localHeight - previousHeight);
      if (height_diff < params[MAX_HEIGHT_DIFFERENCE])
      {
         finalHeight = previousHeight * params[HEIGHT_FILTER_ALPHA] + localHeight * (1.0f - params[HEIGHT_FILTER_ALPHA]);
      }
      else
      {
         // the difference between the incoming data and the old data was too much, reset it to the incoming data completely
         finalHeight = localHeight;

      }
      finalHeight = get_spatial_filtered_height(xIndex, yIndex, finalHeight, globalMap, params);
   }

//   finalHeight = clamp(finalHeight, params[MIN_HEIGHT_REGISTRATION], params[MAX_HEIGHT_REGISTRATION]);
   finalHeight += params[HEIGHT_OFFSET];

   // Put the height value in the global map at the global cell index
   write_imageui(globalMap, (int2)(yIndex, xIndex), (uint4)((int)(finalHeight * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
}

void kernel croppingKernelZUp(read_write image2d_t heightMap,
                              read_write image2d_t croppedMap,
                              global float *params,
                              global float *zUpToWorldFrameTf)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   int2 globalMapSize = (int2) (params[GLOBAL_CELLS_PER_AXIS], params[GLOBAL_CELLS_PER_AXIS]);

   // cell center in world frame
   float3 cellCenterInZUp = (float3)(0, 0, 0);
   cellCenterInZUp.xy = indices_to_coordinate((int2) (xIndex, yIndex),
                                               (float2) (0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
                                               params[GLOBAL_CELL_SIZE],
                                               params[CROPPED_WINDOW_CENTER_INDEX]);

   // transform the point to the ZUp frame
   float3 cellCenterInWorld = transformPoint3D32_2(
      cellCenterInZUp,
      (float3)(zUpToWorldFrameTf[0], zUpToWorldFrameTf[1], zUpToWorldFrameTf[2]),
      (float3)(zUpToWorldFrameTf[4], zUpToWorldFrameTf[5], zUpToWorldFrameTf[6]),
      (float3)(zUpToWorldFrameTf[8], zUpToWorldFrameTf[9], zUpToWorldFrameTf[10]),
      (float3)(zUpToWorldFrameTf[3], zUpToWorldFrameTf[7], zUpToWorldFrameTf[11]));

   // get indices from these coordinates for global map
   int2 globalCellIndex = coordinate_to_indices(
      (float2)(cellCenterInWorld.x, cellCenterInWorld.y),
      (float2)(0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
      params[GLOBAL_CELL_SIZE],
      params[GLOBAL_CENTER_INDEX]);

   // check if global cell index is within bounds
   if (globalCellIndex.x >= 0 && globalCellIndex.x < globalMapSize.x && globalCellIndex.y >= 0 && globalCellIndex.y < globalMapSize.y)
   {
      uint height = read_imageui(heightMap, (int2)(globalCellIndex.y, globalCellIndex.x)).x;
      write_imageui(croppedMap, (int2)(yIndex, xIndex), (uint4)(height, 0, 0, 0));
   }
   else
   {
      write_imageui(croppedMap, (int2)(yIndex, xIndex), (uint4)(0, 0, 0, 0));
   }
}

void kernel croppingKernel(read_write image2d_t inputMap,
                           read_write image2d_t croppedMap,
                           global float *params)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   int2 globalMapSize = (int2) (params[GLOBAL_CELLS_PER_AXIS], params[GLOBAL_CELLS_PER_AXIS]);

   int2 globalSensorIndex = coordinate_to_indices(
      (float2)(params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]),
      (float2)(0, 0), // params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]
      params[GLOBAL_CELL_SIZE],
      params[GLOBAL_CENTER_INDEX]);

   int2 globalCellIndex = (int2) (globalSensorIndex.x + xIndex - params[CROPPED_WINDOW_CENTER_INDEX],
                                    globalSensorIndex.y + yIndex - params[CROPPED_WINDOW_CENTER_INDEX]);

   // check if global cell index is within bounds
   if (globalCellIndex.x >= 0 && globalCellIndex.x < globalMapSize.x && globalCellIndex.y >= 0 && globalCellIndex.y < globalMapSize.y)
   {
      uint height = read_imageui(inputMap, (int2)(globalCellIndex.y, globalCellIndex.x)).x;
      write_imageui(croppedMap, (int2)(yIndex, xIndex), (uint4)(height, 0, 0, 0));
   }
   else
   {
      write_imageui(croppedMap, (int2)(yIndex, xIndex), (uint4)(0, 0, 0, 0));
   }
}

void kernel terrainCostKernel(read_write image2d_t heightMap,
                           read_write image2d_t costMap,
                           global float *params)
{
   // Extract the indices
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   // Compute Sobel operator output for Kx and Ky over the current cell on the height map
   float Kx = 0;
   float Ky = 0;

   // Create a float array to store 3x3 neighborhood of height map
   float heightMapNeighborhood[9];

   // Initialize Sobel operators in two separate arrays
   float KxSobel[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
   float KySobel[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

   // Read in the 3x3 neighborhood of the height map
   if (xIndex > 0 && xIndex < params[GLOBAL_CELLS_PER_AXIS] - 1 && yIndex > 0 && yIndex < params[GLOBAL_CELLS_PER_AXIS] - 1)
   {
      for (int i = -1; i < 2; i++)
      {
         for (int j = -1; j < 2; j++)
         {
            heightMapNeighborhood[(i + 1) * 3 + (j + 1)] = read_imageui(heightMap, (int2) (yIndex + j, xIndex + i)).x / params[HEIGHT_SCALING_FACTOR];
         }
      }

      // Compute Kx and Ky
      for (int i = 0; i < 9; i++)
      {
         Kx += heightMapNeighborhood[i] * KxSobel[i];
         Ky += heightMapNeighborhood[i] * KySobel[i];
      }
   }

   // Compute surface normal from Kx and Ky
   float3 surfaceNormal = (float3)(0, 0, 0);
   surfaceNormal.x = -Kx;
   surfaceNormal.y = -Ky;
   surfaceNormal.z = 1;
   surfaceNormal = normalize(surfaceNormal);

   // Compute the dot product between the surface normal and the z-axis
   float dotProduct = fabs(surfaceNormal.z);

   // Scale-map the dot product to [0, 255] into the cost map in that order
   uint cost = (uint) (dotProduct * 255);

   if (dotProduct < params[STEPPING_COSINE_THRESHOLD])
      cost = 0;

   write_imageui(costMap, (int2)(yIndex, xIndex), (uint4)(cost, 0, 0, 0));
}

void kernel contactMapKernel(read_write image2d_t terrainCost,
                           read_write image2d_t contactMap,
                           global float *params)
{
   int xIndex = get_global_id(0);
   int yIndex = get_global_id(1);

   // Set the contact map at the current cell to be the sum of the 16x16 neighborhood of the terrain cost map
   uint score = 0;
   uint closestDistance = 1000000;
   for (int i = -params[CONTACT_WINDOW_SIZE]; i < params[CONTACT_WINDOW_SIZE]; i++)
   {
      for (int j = -params[CONTACT_WINDOW_SIZE]; j < params[CONTACT_WINDOW_SIZE]; j++)
      {
         if (xIndex + i >= 0 && xIndex + i < params[GLOBAL_CELLS_PER_AXIS] && yIndex + j >= 0 && yIndex + j < params[GLOBAL_CELLS_PER_AXIS])
         {
            uint steppability = read_imageui(terrainCost, (int2) (yIndex + j, xIndex + i)).x;
            if (steppability <= params[STEPPING_CONTACT_THRESHOLD])
            {
               // Chebyshev distance
//               uint distance = max(abs(i), abs(j));

               // Euclidean distance
               uint distance = sqrt((float)(i * i + j * j));

               if (distance < closestDistance && distance > 3.0f)
               {
                  closestDistance = distance;
               }
               else if (distance < 3.0f)
               {
                  closestDistance = 0;
               }
            }
//            score += read_imageui(terrainCost, (int2) (xIndex + i, yIndex + j)).x;
         }
      }
   }
   score = closestDistance;
   write_imageui(contactMap, (int2)(yIndex, xIndex), (uint4)(score, 0, 0, 0));
}

void kernel traversabilityGraphKernel(read_write image2d_t contactMap,
                           read_write image2d_t heightMap,
                           read_write image2d_t traversabilityGraph,
                           global float *params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   int m = 1;

   //    if(cIndex == 0 && rIndex == 0) printf("MergeKernel:(%d)\n", params[MERGE_RANGE]);

   if (rIndex >= m && rIndex < (int) params[GRAPH_CELLS_PER_AXIS] - m && cIndex >= m && cIndex < (int) params[GRAPH_CELLS_PER_AXIS] - m)
   {
      int2 aIndex = (int2) (cIndex, rIndex);

      uint boundaryConnectionsEncodedAsOnes = (uint) (0);

      int count = 0;
      for (int i = -m; i < m + 1; i += m)
      {
         for (int j = -m; j < m + 1; j += m)
         {
            if (!(j == 0 && i == 0))
            {
               int2 bIndex = (int2) (cIndex + i, rIndex + j);

               if (isTraversible(contactMap, heightMap, aIndex, bIndex, params))
               {
                  boundaryConnectionsEncodedAsOnes = (1 << count) | boundaryConnectionsEncodedAsOnes;
               }
               count++;
            }
         }
      }
      write_imageui(traversabilityGraph, (int2) (cIndex, rIndex), (uint4) (123, 0, 0, 0));

      //      printf("MergeKernel[%d,%d] -> (%d)\n", rIndex, cIndex, boundaryConnectionsEncodedAsOnes);
   }
}




float get_yaw_from_index(int yaw_discretizations, int idx_yaw)
{
    return M_PI_F * ((float) idx_yaw) / ((yaw_discretizations - 1));
}

float signed_distance_to_foot_polygon(int center_index, float resolution, global float* params, int2 foot_key, float foot_yaw, int2 query)
{
    int cells_per_side = 2 * center_index + 1;
    int map_idx_x = cells_per_side - query.y;
    int map_idx_y = query.x;
    // we're flipping the axes because it's image vs world
    float2 vector_to_point = resolution * (float2) ((float) (query.x - foot_key.x), (float) (query.y - foot_key.y));

    float2 vector_in_foot_frame = applyYawRotationToVector2D(vector_to_point, -foot_yaw);

    float x_outside = fabs(vector_in_foot_frame.x) - params[SNAP_FOOT_LENGTH] / 2.0f;
    float y_outside = fabs(vector_in_foot_frame.y) - params[SNAP_FOOT_WIDTH] / 2.0f;

    if (x_outside > 0.0f && y_outside > 0.0f)
    {
        return length((float2) (x_outside, y_outside));
    }
    else if (x_outside > 0.0f)
    {
        return x_outside;
    }
    else if (y_outside > 0.0f)
    {
        return y_outside;
    }
    else
    {
        return max(x_outside, y_outside);
    }
}

float signed_distance_to_foot_circle(int center_index, float resolution, global float* params, int2 foot_key, int2 query)
{
    int cells_per_side = 2 * center_index + 1;
    int map_idx_x = cells_per_side - query.y;
    int map_idx_y = query.x;
    // we're flipping the axes because it's image vs world
    float2 vector_to_point = resolution * (float2) ((float) (query.x - foot_key.x), (float) (query.y - foot_key.y));
    float foot_distance = length((float2) (0.5 * params[SNAP_FOOT_LENGTH], 0.5 * params[SNAP_FOOT_WIDTH]));

    return length(vector_to_point) - foot_distance;
}

// This kernel is designed to compute the average snap height for every cell in the window. This can be done by either snapping a rectangular foot down if
// there's a known yaw, or, more efficiently, a circle on the ground, where you don't need to know the yaw. It also computes the local normal at that cell.
// Additionally, it performs some validity checks about the snap, specifically checking the minimum area, or whether it's too close to a cliff top or bottom.
// The results of that check is returned in the steppable map image. When performing the snap, points that are too far below the highest point are ignored. This
// enables a better "sharp" edge around corners, to avound rounding by averaging. It also is how the support area is calculated. In the future, the support area
// should be the area of the convex hull, not just the area of the cells, since that will allow "bridging" gaps.
void kernel computeSnappedValuesKernel(global float* params,
                                read_write image2d_t height_map,
                                global float* idx_yaw_singular_buffer,
                                read_write image2d_t steppable_map,
                                read_write image2d_t snapped_height_map,
                                read_write image2d_t snapped_normal_x_map,
                                read_write image2d_t snapped_normal_y_map,
                                read_write image2d_t snapped_normal_z_map)
{
    // Remember, these are x and y in image coordinates, not world
    int idx_x = get_global_id(0); // column, top left
    int idx_y = get_global_id(1); // row, top left
    int idx_yaw = (int) idx_yaw_singular_buffer[0];

    bool should_print = false;//idx_x == 20 && idx_y == 20;

    int2 key = (int2) (idx_x, idx_y);
    uint foot_height_int = read_imageui(height_map, key).x;

    float foot_height = (float) foot_height_int / params[SNAP_HEIGHT_SCALING_FACTOR] - params[SNAP_HEIGHT_OFFSET];
    float foot_yaw = get_yaw_from_index(2, idx_yaw);

    float foot_width = params[SNAP_FOOT_WIDTH];
    float foot_length = params[SNAP_FOOT_LENGTH];
    float distance_from_bottom = params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];
    float distance_from_top = params[MIN_DISTANCE_FROM_CLIFF_TOPS];

    float map_resolution = params[SNAP_GLOBAL_CELL_SIZE];
    float max_dimension = max(params[SNAP_FOOT_WIDTH], params[SNAP_FOOT_LENGTH]);
    int map_center_index = params[SNAP_GLOBAL_CENTER_INDEX];
    int cropped_center_index = params[SNAP_CROPPED_WINDOW_CENTER_INDEX];
    float2 center = (float2) (params[SNAP_HEIGHT_MAP_CENTER_Y], params[SNAP_HEIGHT_MAP_CENTER_X]);
    float2 map_center = (float2) (0.0, 0.0);

    int map_cells_per_side = 2 * map_center_index + 1;
    int map_cells_per_side_for_checking = map_cells_per_side - 1;
    int cropped_cells_per_side = 2 * cropped_center_index + 1;

    int crop_idx_x = idx_x;
    int crop_idx_y = idx_y;
    int2 crop_key = (int2) (crop_idx_x, crop_idx_y);

    float2 foot_position = indices_to_coordinate(crop_key, center, map_resolution, cropped_center_index);

    // Convert from the world coordinate to the map index.
    int2 map_key = coordinate_to_indices(foot_position, map_center, map_resolution, map_center_index);

    ////// Find the maximum height of any point underneath the foot

    float half_length = foot_length / 2.0f;
    float half_width = foot_width / 2.0f;
    float2 half_foot_size = (float2) (half_length, half_width);
    float foot_search_radius_squared = dot(half_foot_size, half_foot_size);
    float foot_search_radius = sqrt(foot_search_radius_squared);
    int foot_offset_indices = (int) ceil(foot_search_radius / map_resolution);

    int max_height_int = -INFINITY;
    int foot_search_min_x = max(map_key.x - foot_offset_indices, 0);
    int foot_search_max_x = min(map_key.x + foot_offset_indices + 1, map_cells_per_side_for_checking);
    int foot_search_min_y = max(map_key.y - foot_offset_indices, 0);
    int foot_search_max_y = min(map_key.y + foot_offset_indices + 1, map_cells_per_side_for_checking);

    for (int x_query = foot_search_min_x; x_query < foot_search_max_x; x_query++)
    {
        for (int y_query = foot_search_min_y; y_query < foot_search_max_y; y_query++)
        {
            // make sure that this point is within the search radius.
            float2 vector_to_point_from_foot = map_resolution * (float2) ((float) (x_query - map_key.x), (float) (y_query - map_key.y));
            if (dot(vector_to_point_from_foot, vector_to_point_from_foot) > foot_search_radius_squared)
                continue;

            // get the height at this offset point.
            int2 query_key = (int2) (x_query, y_query);
            int query_height_int = read_imageui(height_map, query_key).x;

            max_height_int = max(query_height_int, max_height_int);
        }
    }
    float max_height_under_foot = (float) max_height_int / params[SNAP_HEIGHT_SCALING_FACTOR] - params[SNAP_HEIGHT_OFFSET];

    //////// Compute the local plane of the foot, as well as the area of support underneath it.


    // these are values that will be used for the plane calculation
    float n = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float xx = 0.0f;
    float xy = 0.0f;
    float xz = 0.0f;
    float yy = 0.0f;
    float yz = 0.0f;
    float zz = 0.0f;

    int max_points_possible_under_support;

    int samples = 5;
    float resolution = foot_search_radius / samples;

    max_points_possible_under_support = 0;

    for (int x_value_idx = -samples; x_value_idx <= samples; x_value_idx++)
    {
        for (int y_value_idx = -samples; y_value_idx <= samples; y_value_idx++)
        {
            float2 offset = resolution * (float2) (x_value_idx, y_value_idx);
            float offset_distance_squared = dot(offset, offset);

            if (offset_distance_squared > foot_search_radius_squared)
                continue;

            float offset_distance = sqrt(offset_distance_squared);
            float2 point_query = offset + foot_position;

            int2 query_key = coordinate_to_indices(point_query, map_center, map_resolution, map_center_index);

            // This query position is out of bounds of the incoming height map, so it should be skipped.
            if (query_key.x < 0 || query_key.x > map_cells_per_side_for_checking || query_key.y < 0 || query_key.y > map_cells_per_side_for_checking)
                continue;

            // We want to put this after the bounds check. That way, if it's outside the FOV, we don't count it against the minimum area.
            max_points_possible_under_support++;

            uint query_height_int = read_imageui(height_map, query_key).x;
            float query_height = (float) query_height_int / params[SNAP_HEIGHT_SCALING_FACTOR] - params[SNAP_HEIGHT_OFFSET];

            if (isnan(query_height))
               continue;

            float snap_height_threshold = params[MIN_SNAP_HEIGHT_THRESHOLD] + params[SNAP_HEIGHT_THRESHOLD_AT_SEARCH_EDGE] * clamp(offset_distance / foot_search_radius, 0.0f, 1.0f);
            float min_height_under_foot_to_consider = max_height_under_foot - snap_height_threshold;

            if (isnan(query_height)) // || query_height < min_height_under_foot_to_consider)
               continue;

            // This activation gain is a way of doing a soft inequality. If the query height is less than the min height, as an inequality constraint, the
            // activation value is zero, and if it's greater, the activation is 1.0. In this formulation, we're blurring around that hard inequality. If the
            // query height is less than the min height, the "error" is negative, so the tanh function returns -1.0f. If it's positive, tanh returns 1.0f.
            float tanh_slope = params[INEQUALITY_ACTIVATION_SLOPE];
            float activation = 0.5f * (1.0f + tanh(tanh_slope * (query_height - min_height_under_foot_to_consider)));

            float activation2 = activation * activation;

            n += activation;
            x += activation * point_query.x;
            y += activation * point_query.y;
            z += activation * query_height;
            xx += activation2 * point_query.x * point_query.x;
            xy += activation2 * point_query.x * point_query.y;
            xz += activation2 * point_query.x * query_height;
            yy += activation2 * point_query.y * point_query.y;
            yz += activation2 * point_query.y * query_height;
            zz += activation2 * query_height * query_height;
        }
    }

    ///////////// Solve for the plane normal, as well as the height of the foot along that plane.
    bool failed = false;
    int snap_result = VALID;

    // Fixme this arguably should never happen
    if (n < 0.0001f)
    {
        snap_result = SNAP_FAILED;
        failed = true;
        n = 1.0f;
    }
    // This is the actual height of the snapped foot
    float snap_height = z / n;

    float covariance_matrix[9] = {xx, xy, x, xy, yy, y, x, y, n};
    float z_variance_vector[3] = {-xz, -yz, -z};
    float coefficients[3] = {0.0f, 0.0f, 0.0f};
    solveForPlaneCoefficients(covariance_matrix, z_variance_vector, coefficients);

    float x_solution = x / n;  // average of x positions
    float y_solution = y / n;  // average of y positions
    float z_solution = -coefficients[0] * x_solution - coefficients[1] * y_solution - coefficients[2];

    float3 normal = (float3) (coefficients[0], coefficients[1], 1.0);
    normal = normalize(normal);

    // TODO include this?
    // snap_height = getZOnPlane(foot_position, (float3) (x_solution, y_solution, z_solution), normal);
    int snap_height_int = (snap_height * params[SNAP_HEIGHT_OFFSET]) * params[SNAP_HEIGHT_SCALING_FACTOR];

    /////////////// Make sure there's enough step area.

    float min_points_needed_for_support = (int) (params[MIN_SUPPORT_AREA_FRACTION] * max_points_possible_under_support);
    if (n < min_points_needed_for_support)
    {
        snap_result = NOT_ENOUGH_AREA;
        failed = true;
    }

    //////////// Check to make sure we're not stepping too near a cliff base or top
    if (!failed)
    {
        int cliff_start_height_to_avoid_int = (params[CLIFF_START_HEIGHT_TO_AVOID] + params[SNAP_HEIGHT_OFFSET]) * params[SNAP_HEIGHT_SCALING_FACTOR];
        int cliff_end_height_to_avoid_int = (params[CLIFF_END_HEIGHT_TO_AVOID] + params[SNAP_HEIGHT_OFFSET]) * params[SNAP_HEIGHT_SCALING_FACTOR];

        float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
        float cliff_search_offset_squared = cliff_search_offset * cliff_search_offset;
        int cliff_offset_indices = (int) ceil(cliff_search_offset / map_resolution);
        float min_distance_from_tops = params[MIN_DISTANCE_FROM_CLIFF_TOPS] * params[MIN_DISTANCE_FROM_CLIFF_TOPS];

        int min_x = max(map_key.x - cliff_offset_indices, 0);
        int max_x = min(map_key.x + cliff_offset_indices + 1, map_cells_per_side_for_checking);
        int min_y = max(map_key.y - cliff_offset_indices, 0);
        int max_y = min(map_key.y + cliff_offset_indices + 1, map_cells_per_side_for_checking);

        // search for a cliff base that's too close
        for (int x_query = min_x; x_query < max_x; x_query++)
        {
            for (int y_query = min_y; y_query < max_y; y_query++)
            {
                float2 vector_to_point_from_foot = map_resolution * (float2) ((float) (x_query - map_key.x), (float) (y_query - map_key.y));
                float distance_to_point_squared = dot(vector_to_point_from_foot, vector_to_point_from_foot);
                // skip this cell if it's too far away from the foot, but also skip it if it's within the foot.
                if (distance_to_point_squared > cliff_search_offset_squared || distance_to_point_squared < foot_search_radius_squared)
                    continue;

                // get the height at this offset point.
                int2 query_key = (int2) (x_query, y_query);
                int query_height_int = read_imageui(height_map, query_key).x;

                // compute the relative height at this point, compared to the height contained in the current cell.
                int relative_height_of_query_int = query_height_int - snap_height_int;

                if (relative_height_of_query_int > cliff_start_height_to_avoid_int)
                {
                    float height_alpha = (relative_height_of_query_int - cliff_start_height_to_avoid_int) / (cliff_end_height_to_avoid_int - cliff_start_height_to_avoid_int);
                    height_alpha = clamp(height_alpha, 0.0f, 1.0f);
                    float min_distance_from_this_point_to_avoid_cliff = height_alpha * params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];

                    if (distance_to_point_squared < min_distance_from_this_point_to_avoid_cliff * min_distance_from_this_point_to_avoid_cliff)
                    {
                        // we're too close to the cliff bottom!
                        snap_result = CLIFF_BOTTOM;
                        failed = true;
                        break;
                    }
                }
                else if (relative_height_of_query_int < -cliff_start_height_to_avoid_int)
                {
                    if (distance_to_point_squared < min_distance_from_tops)
                    {
                        // we're too close to the cliff top!
                        snap_result = CLIFF_TOP;
                        failed = true;
                        break;
                    }
                }
            }

            if (failed)
                break;
        }
    }

    int2 storage_key = (int2) (idx_x, idx_y);
    write_imageui(steppable_map, storage_key, (uint4)(snap_result,0,0,0));
    write_imageui(snapped_height_map, storage_key, (uint4)(snap_height_int, 0, 0, 0));
    write_imageui(snapped_normal_x_map, storage_key, (uint4)((int)(normal.x * params[SNAP_HEIGHT_SCALING_FACTOR]), 0, 0, 0));
    write_imageui(snapped_normal_y_map, storage_key, (uint4)((int)(normal.y * params[SNAP_HEIGHT_SCALING_FACTOR]), 0, 0, 0));
    write_imageui(snapped_normal_z_map, storage_key, (uint4)((int)(normal.z * params[SNAP_HEIGHT_SCALING_FACTOR]), 0, 0, 0));
}

void kernel computeSteppabilityConnectionsKernel(global float* params,
                                                 read_only image2d_t steppable_map,
                                                 write_only image2d_t steppable_connections)
{
    int idx_x = get_global_id(0);
    int idx_y = get_global_id(1);

    int cells_per_side = 2 * params[SNAP_CROPPED_WINDOW_CENTER_INDEX] + 1;

    int2 key = (int2) (idx_x, idx_y);

    uint boundaryConnectionsEncodedAsOnes = (uint)(0);

    int counter = 0;
    if (read_imageui(steppable_map, key).x == VALID)
    {
        for (int x_offset = -1; x_offset <= 1; x_offset++)
        {
            for (int y_offset = -1; y_offset <= 1; y_offset++)
            {
                if (x_offset == 0 && y_offset == 0)
                    continue;

                int x_query = idx_x + x_offset;
                int y_query = idx_y + y_offset;

                // out of bounds, so skip it
                if (x_query < 0 || x_query >= cells_per_side || y_query < 0 || y_query >= cells_per_side)
                {
                    boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                }
                else
                {
                    int2 query_key = (int2) (x_query, y_query);
                    if (read_imageui(steppable_map, query_key).x == VALID)
                    {
                        boundaryConnectionsEncodedAsOnes = (1 << counter) | boundaryConnectionsEncodedAsOnes;
                    }
                    else
                    {
                        boundaryConnectionsEncodedAsOnes = (0 << counter) | boundaryConnectionsEncodedAsOnes;
                    }
                }

                counter++;
            }
        }
    }

    write_imageui(steppable_connections, key, (uint4)(boundaryConnectionsEncodedAsOnes, 0, 0, 0));
}