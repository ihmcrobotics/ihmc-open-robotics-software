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