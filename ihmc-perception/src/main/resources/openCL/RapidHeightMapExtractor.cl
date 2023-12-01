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
#define FOOT_LENGTH 330
#define FOOT_WIDTH 34
#define MIN_DISTANCE_FROM_CLIFF_TOPS 35
#define MIN_DISTANCE_FROM_CLIFF_BOTTOMS 36
#define CLIFF_START_HEIGHT_TO_AVOID 37
#define CLIFF_END_HEIGHT_TO_AVOID 38

#define VERTICAL_FOV M_PI_2_F
#define HORIZONTAL_FOV (2.0f * M_PI_F)

#define VALID 0
#define CLIFF_TOP 1
#define CLIFF_BOTTOM 2
#define SNAP_FAILED 3

#define X_TO_PRINT 773
#define Y_TO_PRINT 843

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

    float x_outside = fabs(vector_in_foot_frame.x) - params[FOOT_LENGTH] / 2.0f;
    float y_outside = fabs(vector_in_foot_frame.y) - params[FOOT_WIDTH] / 2.0f;

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
//            if (depth > 2.0f || depth < 0.75f)
//            {
//               continue;
//            }

            float3 queryPointInSensor;
            float3 queryPointInZUp;
            if (params[MODE] == 0) // Spherical
            {
               queryPointInSensor = back_project_spherical(yaw_count, pitch_count, depth, params);
            }
            else if (params[MODE] == 1) // Perspective
            {
               queryPointInSensor = back_project_perspective((int2) (yaw_count, pitch_count), depth, params);
            }

            queryPointInZUp = transformPoint3D32_2(
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
                  if (fabs(queryPointInZUp.z - currentAverageHeight) > 0.05)
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

   int cellsPerAxis = (int)params[LOCAL_CELLS_PER_AXIS];

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
    int idx_x = get_global_id(0); // column
    int idx_y = get_global_id(1); // row
    int idx_yaw = (int) idx_yaw_singular_buffer[0]; // TODO not needed

    int2 key = (int2) (idx_x, idx_y);
    float foot_height = (float) read_imageui(height_map, key).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

    // TODO yaw discretizations should get set
    float foot_yaw = get_yaw_from_index(1, idx_yaw);

    float foot_width = params[FOOT_WIDTH];
    float foot_length = params[FOOT_LENGTH];
    float distance_from_bottom = params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];
    float distance_from_top = params[MIN_DISTANCE_FROM_CLIFF_TOPS];

    float map_resolution = params[GLOBAL_CELL_SIZE];
    float max_dimension = max(params[FOOT_WIDTH], params[FOOT_LENGTH]);
    int center_index = params[GLOBAL_CENTER_INDEX];
    float2 center = (float2) (params[HEIGHT_MAP_CENTER_X], params[HEIGHT_MAP_CENTER_Y]);
    int cells_per_side = 2 * center_index + 1;

    int map_idx_x = cells_per_side - idx_y;
    int map_idx_y = idx_x;
    int2 map_key = (int2) (map_idx_x, map_idx_y);
    float2 foot_position = indices_to_coordinate(map_key, center, map_resolution, center_index);

    // TODO check these
    float cliff_search_offset = max_dimension / 2.0f + max(params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS], params[MIN_DISTANCE_FROM_CLIFF_TOPS]);
    int cliff_offset_indices = (int) ceil(cliff_search_offset / map_resolution);

    // search for a cliff base that's too close
    float max_height = -INFINITY;

    for (int x_query = idx_x - cliff_offset_indices; x_query <= idx_x + cliff_offset_indices; x_query++)
    {
        if (x_query < 0 || x_query >= cells_per_side)
        {
            continue;
        }
        for (int y_query = idx_y - cliff_offset_indices; y_query <= idx_y + cliff_offset_indices; y_query++)
        {
            // y is out of bounds, so skip it
            if (y_query < 0 || y_query >= cells_per_side)
            {
                continue;
            }

            // get the x,y position and height
            int2 query_key = (int2) (x_query, y_query);
            float query_height = (float) read_imageui(height_map, query_key).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

            // compute the relative height at this point
            float relative_height = query_height - foot_height;

            float distance_to_foot = signed_distance_to_foot_polygon(center_index, map_resolution, params, key, foot_yaw, query_key);

            if (relative_height > params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                float height_alpha = (relative_height - params[CLIFF_START_HEIGHT_TO_AVOID]) / (params[CLIFF_END_HEIGHT_TO_AVOID] - params[CLIFF_START_HEIGHT_TO_AVOID]);
                height_alpha = clamp(height_alpha, 0.0f, 1.0f);
                float min_distance_from_this_point = height_alpha * params[MIN_DISTANCE_FROM_CLIFF_BOTTOMS];

                if (min_distance_from_this_point > distance_to_foot)
                {
                    // we're too close to the cliff bottom!
                    // FIXME
      //              write_imageui(steppable_map, key, (uint4)(CLIFF_BOTTOM,0,0,0));

                    return;
                }
            }
            else if (relative_height < -params[CLIFF_START_HEIGHT_TO_AVOID])
            {
                if (params[MIN_DISTANCE_FROM_CLIFF_TOPS] > distance_to_foot)
                {
                    // we're too close to the cliff top!
                    // FIXME
      //              write_imageui(steppable_map, key, (uint4)(CLIFF_TOP,0,0,0));

                    return;
                }
            }

            //  TODO extract epsilon
            if (distance_to_foot < 1e-3f)
            {
                max_height = max(query_height, max_height);
            }
        }
    }

    int points_inside_polygon = 0;
    float running_height_total = 0.0f;
    float min_height = max_height - 0.05f;
    float resolution = 0.02f;
    float half_length = foot_length / 2.0f;
    float half_width = foot_width / 2.0f;

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

    for (float x_value = -half_length; x_value <= half_length; x_value += resolution)
    {
        for (float y_value = -half_width; y_value <= half_width; y_value += resolution)
        {
            float2 vector_in_foot = applyYawRotationToVector2D((float2) (x_value, y_value), foot_yaw);
            float2 point_query = vector_in_foot + foot_position;

            int map_query_x = coordinate_to_index(point_query.x, center.x, map_resolution, center_index);
            int map_query_y = coordinate_to_index(point_query.y, center.y, map_resolution, center_index);
            int image_query_x = map_query_y;
            int image_query_y = cells_per_side - map_query_x;
            int2 query_key = (int2) (image_query_x, image_query_y);
            float query_height = (float) read_imageui(height_map, query_key).x / params[HEIGHT_SCALING_FACTOR] - params[HEIGHT_OFFSET];

            if (isnan(query_height) || query_height < min_height)
               continue;

 //        //  if (query_height > params[HEIGHT_OFFSET])
 //        //  {
 //        //      printf("Coordinates %d, di\n", idx_x, idx_y);
 //        //  }
//
 //        //  if (idx_x == X_TO_PRINT && idx_y == Y_TO_PRINT)
 //        //  {
 //        //      printf("query is inside the foot at height %f\n", query_height);
 //        //  }
//
            points_inside_polygon++;
            running_height_total += query_height;

            n += 1.0f;
            x += point_query.x;
            y += point_query.y;
            z += query_height;
            xx += point_query.x * point_query.x;
            xy += point_query.x * point_query.y;
            xz += point_query.x * query_height;
            yy += point_query.y * point_query.y;
            yz += point_query.y * query_height;
            zz += query_height * query_height;
        }
    }

    int lengths = (int) floor(foot_length / resolution);
    int widths = (int) floor(foot_width / resolution);
    int max_points = lengths * widths;

    // TODO extract area fraction
    float min_area_fraction = 0.75f;
    int min_points = (int) (min_area_fraction * max_points);
    if (points_inside_polygon > min_points)
    {
        float snap_height = running_height_total / points_inside_polygon;

        float covariance_matrix[9] = {xx, xy, x, xy, yy, y, x, y, n};
        float z_variance_vector[3] = {-xz, -yz, -z};
        float coefficients[3] = {0, 0, 0};
        solveForPlaneCoefficients(covariance_matrix, z_variance_vector, coefficients);

        float x_solution = x / n;
        float y_solution = y / n;
        float z_solution = -coefficients[0] * x_solution - coefficients[1] * y_solution - coefficients[2];

     //   float3 normal = (float3) (coefficients[0], coefficients[1], 1.0);
     //   normal = normalize(normal);

      //  if (idx_x == X_TO_PRINT && idx_y == Y_TO_PRINT)
      //  {
      //      printf("running_height_total %f\n", running_height_total);
      //      printf("snap height %f\n", snap_height);
      //  }

    // FIXME
      // write_imageui(steppable_map, key, (uint4)(VALID,0,0,0));


      // write_imageui(snapped_height_map, key, (uint4)((int)((snap_height + params[HEIGHT_OFFSET]) * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
      // write_imageui(snapped_normal_x_map, key, (uint4)((int)(normal.x * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
      // write_imageui(snapped_normal_y_map, key, (uint4)((int)(normal.y * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
      // write_imageui(snapped_normal_z_map, key, (uint4)((int)(normal.z * params[HEIGHT_SCALING_FACTOR]), 0, 0, 0));
    }
    else
    {
    // fixme

   //    write_imageui(steppable_map, key, (uint4)(SNAP_FAILED,0,0,0));
   //    write_imageui(snapped_height_map, key, (uint4)(0, 0, 0, 0));
   //    write_imageui(snapped_normal_x_map, key, (uint4)(0, 0, 0, 0))
   //    write_imageui(snapped_normal_y_map, key, (uint4)(0, 0, 0, 0));
   //    write_imageui(snapped_normal_z_map, key, (uint4)(0, 0, 0, 0));
    }
}

void kernel croppingKernel(read_write image2d_t heightMap,
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

               if (distance < closestDistance)
               {
                  closestDistance = distance;
               }
            }
//            score += read_imageui(terrainCost, (int2) (xIndex + i, yIndex + j)).x;
         }
      }
   }
   score = closestDistance;
   write_imageui(contactMap, (int2)(yIndex, xIndex), (uint4)(score, 0, 0, 0));
}
