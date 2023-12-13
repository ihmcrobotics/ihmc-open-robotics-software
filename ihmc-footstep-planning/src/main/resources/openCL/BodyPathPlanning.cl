#define FLOAT_TO_INT_SCALE 10000
#define EPS_ANGLE_SHIFT 1.0e-12;

// These are the height map parameters
#define HEIGHT_MAP_RESOLUTION 0
#define CENTER_INDEX 1
#define centerX 2
#define centerY 3
#define SNAP_HEIGHT_THRESHOLD_FOR_LEAST_SQUARES 4
#define GROUND_HEIGHT_ESTIMATE 5

// these are the path planning parameters
#define PATH_RESOLUTION 0
#define PATH_CENTER_INDEX 1
#define START_X_INDEX 2
#define START_Y_INDEX 3
#define GOAL_X 4
#define GOAL_Y 5
#define NOMINAL_INCLINE 6
#define CHECK_FOR_COLLISIONS 7
#define COMPUTE_SURFACE_NORMALS 8
#define COMPUTE_TRAVERSIBILITY 9
#define MIN_SNAP_HEIGHT_THRESHOLD 10
#define GROUND_CLEARANCE 11
#define MAX_INCLINE 12
#define INCLINE_COST_WEIGHT 13
#define INCLINE_COST_DEADBAND 14
#define ROLL_COST_WEIGHT 15
#define ROLL_DEADBAND 16
#define MAX_PENALIZED_ROLL_ANGLE 17
#define TRAVERSIBILITY_STANCE_ALPHA 18
#define TRAVERSIBILITY_STEP_ALPHA 19
#define MIN_TRAVERSIBILITY_PERCENTAGE 20
#define HALF_STANCE_WIDTH 21
#define TRAVERSIBILITY_HEIGHT_WINDOW 22
#define TRAVERSIBILITY_MIN_NORMAL_TO_PENALIZE 23
#define TRAVERSIBILITY_MAX_NORMAL_TO_PENALIZE 24
#define TRAVERSIBILITY_INCLINE_WEIGHT 25
#define TRAVERSIBILITY_WEIGHT 26
#define TRAVERSIBILITY_HEIGHT_DEADBAND 27
#define TRAVERSIBILITY_HEIGHT_PROXIMITY_FOR_SAYING_WALKING_ON_GROUND 28
#define TRAVERSIBILITY_LOWEST_NON_GROUND_DISCOUNT_WHEN_WALKING_ON_GROUND 29
#define MINIMUM_CELLS_FOR_TRAVERSIBLE 30

// These are the flags for the different rejection types for the edges
#define VALID -1
#define INVALID_SNAP 0
#define TOO_STEEP 1
#define STEP_TOO_HIGH 2
#define COLLISION 3
#define NON_TRAVERSIBLE 4

// These are the variables for the smoother
#define NUMBER_OF_WAYPOINTS 0
#define EQUAL_SPACING_WEIGHT 1
#define MIN_CURVATURE_TO_PENALIZE 2
#define GRADIENT_EPSILON 3
#define SMOOTHNESS_WEIGHT 4
#define BOX_SIZE_X 5
#define BOX_SIZE_Y 6
#define BOX_GROUND_OFFSET 7
#define COLLISION_WEIGHT 8
#define YAW_DISCRETIZATIONS 9
#define FLAT_GROUND_WEIGHT 10
#define TRAVERSIBILITY_SMOOTHING_HEIGHT_DEADBAND 11
#define TURN_POINT_SMOOTHNESS_DISCOUNT 12
#define SMOOTHER_TRAVERSIBILITY_THRESHOLD 13
#define TRAVERSIBILITY_THRESHOLD_FOR_NO_DISCOUNT 14

// these are parameters defined explicitly for the RANSAC normal calculator
#define RANSAC_ITERATIONS 0
#define RANSAC_DISTANCE_EPSILON 1
#define RANSAC_MIN_NORMAL_Z 2
#define RANSAC_ACCEPTABLE_CONSENSUS 3

float index_to_yaw(int yaw_index, int yaw_total_indices)
{
   int center_index = yaw_total_indices / 2;
   return M_2_PI_F * (yaw_index - center_index) / yaw_total_indices;
}

int yaw_to_index(float yaw, int yaw_total_indices)
{
   return ((int) (yaw / yaw_total_indices)) + (yaw_total_indices / 2);
}

void kernel computeSurfaceNormalsWithRANSAC(global float* params,
                                            global float* ransac_params,
                                            global int* ransac_offsets,
                                            global float* height_map,
                                            global float* normal_xyz_buffer)
{
   int key = get_global_id(0);

   int center_index = (int) params[CENTER_INDEX];
   int cells_per_side = 2 * center_index + 1;

   uint seed = key;

   int idx_x = key_to_x_index(key, center_index);
   int idx_y = key_to_y_index(key, center_index);

   float x_coordinate = index_to_coordinate(idx_x, params[centerX], params[HEIGHT_MAP_RESOLUTION], center_index);
   float y_coordinate = index_to_coordinate(idx_y, params[centerY], params[HEIGHT_MAP_RESOLUTION], center_index);

   float3 point = (float3) (x_coordinate, y_coordinate, height_map[key]);

   int xOffset0, yOffset0, xOffset1, yOffset1;
   int xIndex0, yIndex0, xIndex1, yIndex1;

   int maxConsensus = -1;

   int offsets_size = (int) ransac_offsets[0];
   int consensus_size = ransac_offsets[1];
   float3 best_normal;

   int consensusSampleSize = offsets_size;

   for (int i = 0; i < ransac_params[RANSAC_ITERATIONS]; i++)
   {
      while (true)
      {
         // FIXME make this use the correct size
         uint2 ret = nextRandomInt(seed, offsets_size);
         uint sample0 = ret.s0;
         seed = ret.s1;

         xOffset0 = ransac_offsets[2 + sample0];
         yOffset0 = ransac_offsets[2 + offsets_size + sample0];

         xIndex0 = idx_x + xOffset0;
         yIndex0 = idx_y + yOffset0;

         // TODO skip if we've already done that sample
         if (xIndex0 < 0 || xIndex0 >= cells_per_side || yIndex0 < 0 || yIndex0 >= cells_per_side)
            continue;

         // FIXME make this use the correct size
         ret = nextRandomInt(seed, offsets_size);
         uint sample1 = ret.s0;
         seed = ret.s1;

         // FIXME make this use the correct size
         sample0 = sample0 % offsets_size;
         sample1 = sample1 % offsets_size;

         if (sample0 == sample1)
         {
            continue;
         }

         xOffset1 = ransac_offsets[2 + sample1];
         yOffset1 = ransac_offsets[2 + offsets_size + sample1];

         xIndex1 = idx_x + xOffset1;
         yIndex1 = idx_y + yOffset1;

         if (xIndex1 < 0 || xIndex1 >= cells_per_side || yIndex1 < 0 || yIndex1 >= cells_per_side)
         {
            continue;
         }

         float2 offset0 = (float2) (xOffset0, yOffset0);
         float2 offset1 = (float2) (xOffset1, yOffset1);
         float dotProduct = dot(offset0, offset1) / (length(offset0) * length(offset1));

         if (epsilonEquals(fabs(dotProduct), 1.0f, 1e-5))
         {
            continue;
         }

         break;
      }

      // get the 3D points that correspond to the two random samples
      int key0 = indices_to_key(xIndex0, yIndex0, center_index);
      float x_coordinate0 = index_to_coordinate(xIndex0, params[centerX], params[HEIGHT_MAP_RESOLUTION], center_index);
      float y_coordinate0 = index_to_coordinate(yIndex0, params[centerY], params[HEIGHT_MAP_RESOLUTION], center_index);
      float z_coordinate0 = height_map[key0];
      float3 point0 = (float3) (x_coordinate0, y_coordinate0, z_coordinate0);

      int key1 = indices_to_key(xIndex1, yIndex1, center_index);
      float x_coordinate1 = index_to_coordinate(xIndex1, params[centerX], params[HEIGHT_MAP_RESOLUTION], center_index);
      float y_coordinate1 = index_to_coordinate(yIndex1, params[centerY], params[HEIGHT_MAP_RESOLUTION], center_index);
      float z_coordinate1 = height_map[key1];
      float3 point1 = (float3) (x_coordinate1, y_coordinate1, z_coordinate1);

      // compute the normal from the triple of points
      float3 candidate_normal = computeNormal3DFromThreePoint3Ds(point, point0, point1);

      // if the normal is too vertical, keep looking
      if (fabs(candidate_normal.s2) < ransac_params[RANSAC_MIN_NORMAL_Z])
      {
         continue;
      }

      int consensus = 0;
      int offsetsStart = 2 + 2 * offsets_size;
      for (int j = 0; j < consensus_size; j++)
      {
         int xj = idx_x + ransac_offsets[offsetsStart + j];
         int yj = idx_y + ransac_offsets[offsetsStart + consensus_size + j];
         if (xj < 0 || xj >= cells_per_side || yj < 0 || yj >= cells_per_side)
         {
            continue;
         }

         // get the neighboring point
         int keyConsensus = indices_to_key(xj, yj, center_index);
         float x_consensus = index_to_coordinate(xj, params[centerX], params[HEIGHT_MAP_RESOLUTION], center_index);
         float y_consensus = index_to_coordinate(yj, params[centerY], params[HEIGHT_MAP_RESOLUTION], center_index);
         float z_consensus = height_map[keyConsensus];
         float3 consensus_point = (float3) (x_consensus, y_consensus, z_consensus);

         // check the distance of the neighboring points to the plane
         float distanceToPlane = distanceFromPoint3DToPlane3D(consensus_point, point, candidate_normal);

         // if the distance is less than an epsilon, we can say that point is on the plane.
         if (distanceToPlane < ransac_params[RANSAC_DISTANCE_EPSILON])
         {
            consensus++;
         }
      }

      // if this plane has more matching points, it's a better option
      if (consensus > maxConsensus)
      {
         maxConsensus = consensus;
         best_normal = candidate_normal;
      }

      // if we have enough matching points, terminate the search
      if (maxConsensus > ransac_params[RANSAC_ACCEPTABLE_CONSENSUS] * consensusSampleSize)
      {
         break;
      }
   }

   // if the normal is upside down, negate it.
   if (best_normal.s2 < 0.0f)
   {
      best_normal = -best_normal;
   }

   normal_xyz_buffer[3 * key] = best_normal.s0;
   normal_xyz_buffer[3 * key + 1] = best_normal.s1;
   normal_xyz_buffer[3 * key + 2] = best_normal.s2;
}

void kernel computeSurfaceNormalsWithLeastSquares(
    global float* params, global int* offsets, global float* height_map, global float* normal_xyz_buffer, global float* sampled_height)
{
   int key = get_global_id(0);

   int center_index = (int) params[CENTER_INDEX];

   int idx_x = key_to_x_index(key, center_index);
   int idx_y = key_to_y_index(key, center_index);

   int cells_per_side = 2 * center_index + 1;

   int connections = offsets[0];

   // compute the maximum height in the area of interest.
   float max_z = -INFINITY;
   for (int cell = 0; cell < connections; cell++)
   {
      int idx_x_to_poll = idx_x + offsets[cell + 1];
      int idx_y_to_poll = idx_y + offsets[connections + cell + 1];

      // check to make sure the index is in frame
      if (idx_x_to_poll < 0 || idx_x_to_poll >= cells_per_side)
         continue;
      if (idx_y_to_poll < 0 || idx_y_to_poll >= cells_per_side)
         continue;

      int key_to_poll = indices_to_key(idx_x_to_poll, idx_y_to_poll, center_index);
      float z_coordinate = height_map[key_to_poll];

      if (!isnan(z_coordinate))
         max_z = max(max_z, z_coordinate);
   }

   // fit a plane to points in the patch, which are a three by three grid.
   float min_z = max_z - params[SNAP_HEIGHT_THRESHOLD_FOR_LEAST_SQUARES];

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

   for (int cell = 0; cell < connections; cell++)
   {
      int idx_x_to_poll = idx_x + offsets[cell + 1];
      int idx_y_to_poll = idx_y + offsets[connections + cell + 1];

      if (idx_x_to_poll < 0 || idx_x_to_poll >= cells_per_side)
         continue;
      if (idx_y_to_poll < 0 || idx_y_to_poll >= cells_per_side)
         continue;

      int key_to_poll = indices_to_key(idx_x_to_poll, idx_y_to_poll, center_index);
      float z_coordinate_to_poll = height_map[key_to_poll];

      if (isnan(z_coordinate_to_poll) || z_coordinate_to_poll < min_z)
         continue;

      float x_coordinate_to_poll = index_to_coordinate(idx_x_to_poll, params[centerX], params[HEIGHT_MAP_RESOLUTION], center_index);
      float y_coordinate_to_poll = index_to_coordinate(idx_y_to_poll, params[centerY], params[HEIGHT_MAP_RESOLUTION], center_index);

      n += 1.0f;
      x += x_coordinate_to_poll;
      y += y_coordinate_to_poll;
      z += z_coordinate_to_poll;
      xx += x_coordinate_to_poll * x_coordinate_to_poll;
      xy += x_coordinate_to_poll * y_coordinate_to_poll;
      xz += x_coordinate_to_poll * z_coordinate_to_poll;
      yy += y_coordinate_to_poll * y_coordinate_to_poll;
      yz += y_coordinate_to_poll * z_coordinate_to_poll;
      zz += z_coordinate_to_poll * z_coordinate_to_poll;
   }

   float3 normal;
   float height_at_center;

   if (n > 3)
   {
      float covariance_matrix[9] = {xx, xy, x, xy, yy, y, x, y, n};
      float z_variance_vector[3] = {-xz, -yz, -z};
      float coefficients[3] = {0, 0, 0};
      solveForPlaneCoefficients(covariance_matrix, z_variance_vector, coefficients);

      float x_solution = x / n;
      float y_solution = y / n;
      float z_solution = -coefficients[0] * x_solution - coefficients[1] * y_solution - coefficients[2];

      normal = (float3) (coefficients[0], coefficients[1], 1.0);
      normal = normalize(normal);

      float x_coordinate = index_to_coordinate(idx_x, params[centerX], params[HEIGHT_MAP_RESOLUTION], center_index);
      float y_coordinate = index_to_coordinate(idx_y, params[centerY], params[HEIGHT_MAP_RESOLUTION], center_index);
      float z_coordinate = height_map[key];

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      height_at_center = normal.s0 / normal.s2 * (x_solution - x_coordinate) + normal.s1 / normal.s2 * (y_solution - y_coordinate) + z_coordinate;

      if (isnan(normal.s0))
      {
         normal = (float3) (0.0f, 0.0f, 1.0f);
         height_at_center = NAN;
      }
   }
   else
   {
      normal = (float3) (0.0f, 0.0f, 1.0f);
      height_at_center = NAN;
   }

   normal_xyz_buffer[3 * key] = normal.s0;
   normal_xyz_buffer[3 * key + 1] = normal.s1;
   normal_xyz_buffer[3 * key + 2] = normal.s2;
   sampled_height[key] = height_at_center;
}

void kernel snapVertices(
    global float* height_map_params, global float* planner_params, global float* height_map, global int* neighbor_offsets, global float* snapped_vertex_height)
{
   int path_key = get_global_id(0);

   int map_center_index = (int) height_map_params[CENTER_INDEX];
   float map_resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   int path_center_index = (int) planner_params[PATH_CENTER_INDEX];
   float path_resolution = planner_params[PATH_RESOLUTION];

   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);

   int path_idx_x = key_to_x_index(path_key, path_center_index);
   int path_idx_y = key_to_y_index(path_key, path_center_index);
   int2 path_idx = (int2) (path_idx_x, path_idx_y);

   float2 node_location = indices_to_coordinate(path_idx, center, path_resolution, path_center_index);

   int map_x_index = coordinate_to_index(node_location.x, center.x, map_resolution, map_center_index);
   int map_y_index = coordinate_to_index(node_location.y, center.y, map_resolution, map_center_index);

   int number_of_neighbors = neighbor_offsets[0];
   int map_cells_per_side = 2 * map_center_index + 1;

   // compute the max z
   float max_z = -INFINITY;
   for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
   {
      int neighbor_idx_x = map_x_index + neighbor_offsets[1 + neighborIdx];
      int neighbor_idx_y = map_y_index + neighbor_offsets[1 + number_of_neighbors + neighborIdx];

      if (neighbor_idx_x < 0 || neighbor_idx_x >= map_cells_per_side)
      {
         continue;
      }
      if (neighbor_idx_y < 0 || neighbor_idx_y >= map_cells_per_side)
      {
         continue;
      }

      int neighbor_map_key = indices_to_key(neighbor_idx_x, neighbor_idx_y, map_center_index);
      float neighbor_z = height_map[neighbor_map_key];

      if (!isnan(neighbor_z))
         max_z = max(max_z, neighbor_z);
   }

   float height_sample_delta = planner_params[MIN_SNAP_HEIGHT_THRESHOLD];
   float min_z = max_z - height_sample_delta;

   // compute the average z height
   float running_sum = 0;
   int number_of_samples = 0;
   for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
   {
      int neighbor_idx_x = map_x_index + neighbor_offsets[1 + neighborIdx];
      int neighbor_idx_y = map_y_index + neighbor_offsets[1 + number_of_neighbors + neighborIdx];

      if (neighbor_idx_x < 0 || neighbor_idx_x >= map_cells_per_side)
      {
         continue;
      }
      if (neighbor_idx_y < 0 || neighbor_idx_y >= map_cells_per_side)
      {
         continue;
      }

      int neighbor_map_key = indices_to_key(neighbor_idx_x, neighbor_idx_y, map_center_index);
      float neighbor_z = height_map[neighbor_map_key];

      if (!isnan(neighbor_z) && neighbor_z > min_z)
      {
         running_sum += neighbor_z;
         number_of_samples++;
      }
   }

   snapped_vertex_height[path_key] = running_sum / number_of_samples;
}

float get_yaw(int yaw_index)
{
   return yaw_index * M_PI_4_F / 2.0f;
}

int computeCollisionOffsetX(int yaw_idx, int x_offset, int y_offset)
{
   // Box is symmetric so treat rotation by pi the same;
   int yawIndexMod = yaw_idx % 8;

   if (yawIndexMod == 0 || yawIndexMod == 1 || yawIndexMod == 2)
   { // rotate by 0
      return x_offset;
   }
   else if (yawIndexMod == 3)
   { // reflect across x = y
      return y_offset;
   }
   else if (yawIndexMod == 4 || yawIndexMod == 5 || yawIndexMod == 6)
   { // rotate by 90
      return -y_offset;
   }
   else if (yawIndexMod == 7)
   { // reflect across x axis
      return x_offset;
   }

   printf("Invalid yaw index %d\n", yaw_idx);
   return 0;
}

int computeCollisionOffsetY(int yaw_idx, int x_offset, int y_offset)
{
   // Box is symmetric so treat rotation by pi the same;
   int yawIndexMod = yaw_idx % 8;

   if (yawIndexMod == 0 || yawIndexMod == 1 || yawIndexMod == 2)
   { // rotate by 0
      return y_offset;
   }
   else if (yawIndexMod == 3)
   { // reflect across x = y
      return x_offset;
   }
   else if (yawIndexMod == 4 || yawIndexMod == 5 || yawIndexMod == 6)
   { // rotate by 90
      return x_offset;
   }
   else if (yawIndexMod == 7)
   { // reflect across x axis
      return -y_offset;
   }

   printf("Invalid yaw index %d\n", yaw_idx);
   return 0;
}

int getTraversibilityOffsetSet(int yaw_index)
{
   if (yaw_index == 0 || yaw_index == 2 || yaw_index == 4 || yaw_index == 6)
   {
      return 0;
   }
   else if (yaw_index == 1 || yaw_index == 3 || yaw_index == 5 || yaw_index == 7)
   {
      return 1;
   }
   else
   {
      return 2;
   }
}

float computeSidedTraversibility(global float* height_map_params,
                                 global float* planner_params,
                                 global int* offsets,
                                 int offset_set,
                                 global float* height_map_data,
                                 global float* normal_xyz_data,
                                 int node_side,
                                 float2 node,
                                 int neighbor_idx,
                                 float opposite_height,
                                 float nominal_height)
{
   float half_stance_width = planner_params[HALF_STANCE_WIDTH];
   if (node_side == 1)
      half_stance_width = -half_stance_width;

   float yaw = get_yaw(neighbor_idx);
   float2 local_offset = (float2) (0.0f, half_stance_width);
   float2 translation = applyInverseYawRotationToVector2D(local_offset, yaw);
   node += translation;

   int center_index = (int) height_map_params[CENTER_INDEX];
   int cells_per_side = 2 * center_index + 1;

   int x_index = coordinate_to_index(node.x, height_map_params[centerX], height_map_params[HEIGHT_MAP_RESOLUTION], center_index);
   int y_index = coordinate_to_index(node.y, height_map_params[centerY], height_map_params[HEIGHT_MAP_RESOLUTION], center_index);

   int numberOfSampledCells = 0;
   int numberOfTraversibleCells = 0;

   float traversibilityScoreNumber = 0.0f;
   float minHeight = max(opposite_height, nominal_height) - planner_params[TRAVERSIBILITY_HEIGHT_WINDOW];
   float maxHeight = min(opposite_height, nominal_height) + planner_params[TRAVERSIBILITY_HEIGHT_WINDOW];
   float averageHeight = 0.5f * (nominal_height + opposite_height);
   float windowWidth = (maxHeight - minHeight) / 2.0f;

   float lowestNonGroundAlpha = planner_params[TRAVERSIBILITY_LOWEST_NON_GROUND_DISCOUNT_WHEN_WALKING_ON_GROUND];
   float heightAboveGround = fabs(averageHeight - height_map_params[GROUND_HEIGHT_ESTIMATE]);
   float discountForNonGroundPointsWhenWalkingOnGround = 1.0f;
   float groundProximity = planner_params[TRAVERSIBILITY_HEIGHT_PROXIMITY_FOR_SAYING_WALKING_ON_GROUND];
   if (heightAboveGround < groundProximity)
   {
      discountForNonGroundPointsWhenWalkingOnGround = lowestNonGroundAlpha + (1.0f - lowestNonGroundAlpha) * heightAboveGround / groundProximity;
   }

   // cell is not traversible
   if (minHeight > maxHeight - 1e-3)
      return 0.0f;

   // get the location in the offsets vector to pull from
   int number_of_offsets = offsets[offset_set];
   int x_offset_start = 3;
   int y_offset_start = x_offset_start + offsets[0];
   for (int i = 0; i < offset_set; i++)
   {
      x_offset_start += 2 * offsets[i];
      y_offset_start += offsets[i] + offsets[i + 1];
   }

   for (int i = 0; i < number_of_offsets; i++)
   {
      int xOffset = offsets[x_offset_start + i];
      int yOffset = offsets[y_offset_start + i];
      int xQuery = x_index + computeCollisionOffsetX(i, xOffset, yOffset);
      int yQuery = y_index + computeCollisionOffsetY(i, xOffset, yOffset);

      if (xQuery < 0 || yQuery < 0 || xQuery >= cells_per_side || yQuery >= cells_per_side)
         continue;

      int query_key = indices_to_key(xQuery, yQuery, center_index);
      float heightQuery = height_map_data[query_key];

      numberOfSampledCells++;
      if (heightQuery > minHeight && heightQuery < maxHeight)
      {
         numberOfTraversibleCells++;

         float deltaHeight = max(0.0f, fabs(averageHeight - heightQuery) - planner_params[TRAVERSIBILITY_HEIGHT_DEADBAND]);
         float cellPercentage = 1.0f - deltaHeight / windowWidth;
         float nonGroundDiscount = 1.0f;

         if (!epsilonEquals(heightQuery, height_map_params[GROUND_HEIGHT_ESTIMATE], 1e-3))
         {
            nonGroundDiscount = discountForNonGroundPointsWhenWalkingOnGround;
         }

         float query_normal_x = normal_xyz_data[3 * query_key];
         float query_normal_y = normal_xyz_data[3 * query_key + 1];
         float query_normal_z = normal_xyz_data[3 * query_key + 2];
         float incline = max(0.0f, acos(query_normal_z) - planner_params[TRAVERSIBILITY_MIN_NORMAL_TO_PENALIZE]);
         float inclineAlpha = (planner_params[TRAVERSIBILITY_MAX_NORMAL_TO_PENALIZE] - incline) /
                              (planner_params[TRAVERSIBILITY_MAX_NORMAL_TO_PENALIZE] - planner_params[TRAVERSIBILITY_MIN_NORMAL_TO_PENALIZE]);
         inclineAlpha = clamp(inclineAlpha, 0.0f, 1.0f);
         traversibilityScoreNumber += nonGroundDiscount * ((1.0f - planner_params[TRAVERSIBILITY_INCLINE_WEIGHT]) * cellPercentage +
                                                           planner_params[TRAVERSIBILITY_INCLINE_WEIGHT] * inclineAlpha);
      }
   }

   if (numberOfSampledCells < planner_params[MINIMUM_CELLS_FOR_TRAVERSIBLE])
   {
      return 0.0f;
   }
   else
   {
      float traversibility = traversibilityScoreNumber / numberOfSampledCells;
      return traversibility;
   }
}

float4 computeTraversibilityMeasures(global float* height_map_params,
                                     global float* planner_params,
                                     global int* traversibility_offsets,
                                     global float* snapped_vertex_height,
                                     global float* height_map_data,
                                     global float* normal_xyz_data,
                                     float2 parent_node,
                                     float2 child_node,
                                     int yaw_idx)
{
   int path_center_index = planner_params[PATH_CENTER_INDEX];

   float2 grid_center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   int parent_key = coordinate_to_key(parent_node, grid_center, planner_params[PATH_RESOLUTION], path_center_index);
   int child_key = coordinate_to_key(child_node, grid_center, planner_params[PATH_RESOLUTION], path_center_index);
   int parent_planner_x = (int) round(parent_node.x / planner_params[PATH_RESOLUTION]);
   int parent_planner_y = (int) round(parent_node.y / planner_params[PATH_RESOLUTION]);

   float parent_height = snapped_vertex_height[parent_key];
   float child_height = snapped_vertex_height[child_key];
   int offset_set = getTraversibilityOffsetSet(yaw_idx);

   float left_traversibility = computeSidedTraversibility(height_map_params, planner_params, traversibility_offsets, offset_set, height_map_data,
                                                          normal_xyz_data, 0, child_node, yaw_idx, parent_height, child_height);
   float right_traversibility = computeSidedTraversibility(height_map_params, planner_params, traversibility_offsets, offset_set, height_map_data,
                                                           normal_xyz_data, 1, child_node, yaw_idx, parent_height, child_height);

   float previous_left_traversibility = 1.0f;
   float previous_right_traversibility = 1.0f;
   if (parent_planner_x != planner_params[START_X_INDEX] && parent_planner_y != planner_params[START_Y_INDEX])
   {
      previous_left_traversibility = computeSidedTraversibility(height_map_params, planner_params, traversibility_offsets, offset_set, height_map_data,
                                                                normal_xyz_data, 0, parent_node, yaw_idx, child_height, parent_height);
      previous_right_traversibility = computeSidedTraversibility(height_map_params, planner_params, traversibility_offsets, offset_set, height_map_data,
                                                                 normal_xyz_data, 1, parent_node, yaw_idx, child_height, parent_height);
   }

   float leftStepScore = sqrt(left_traversibility * previous_right_traversibility);
   float rightStepScore = sqrt(right_traversibility * previous_left_traversibility);

   return (float4) (left_traversibility, right_traversibility, leftStepScore, rightStepScore);
}

float computeTraversibilityCost(global float* planner_params, float4 traversibility_measures)
{
   float stanceTraversibility = max(traversibility_measures.s0, traversibility_measures.s1);
   float stepTraversibility = max(traversibility_measures.s2, traversibility_measures.s3);

   return planner_params[TRAVERSIBILITY_WEIGHT] * (planner_params[TRAVERSIBILITY_STANCE_ALPHA] * (1.0f - stanceTraversibility) +
                                                   planner_params[TRAVERSIBILITY_STEP_ALPHA] * (1.0f - stepTraversibility));
}

bool computeIsTraversible(global float* planner_params, float4 traversibility_measures)
{
   return traversibility_measures.s0 > planner_params[MIN_TRAVERSIBILITY_PERCENTAGE] ||
          traversibility_measures.s1 > planner_params[MIN_TRAVERSIBILITY_PERCENTAGE];
}

float computeRollAtNode(global float* height_map_params, global float* normal_least_squares_xyz_buffer, float2 parent, float2 neighbor)
{
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   int map_center_index = height_map_params[CENTER_INDEX];
   float map_resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   float2 edge = normalize(neighbor - parent);
   float2 averageBodyPosition = (neighbor + parent) / 2.0f;

   int body_map_key = coordinate_to_key(averageBodyPosition, center, map_resolution, map_center_index);
   float normal_x = normal_least_squares_xyz_buffer[3 * body_map_key];
   float normal_y = normal_least_squares_xyz_buffer[3 * body_map_key + 1];
   float normal_z = normal_least_squares_xyz_buffer[3 * body_map_key + 2];

   float roll = asin(fabs(edge.y * normal_x - edge.x * normal_y));
   return roll;
}

float computeRollCost(global float* planner_params, float incline, float roll)
{
   float maxPenalizedRoll = planner_params[MAX_PENALIZED_ROLL_ANGLE] - planner_params[ROLL_DEADBAND];
   float inclineScale = clamp(fabs(incline) / maxPenalizedRoll, 0.0f, 1.0f);
   float deadbandedRollAngle = max(0.0f, fabs(roll) - planner_params[ROLL_DEADBAND]);

   return planner_params[ROLL_COST_WEIGHT] * inclineScale * deadbandedRollAngle;
}

int get_collision_set_index(int yaw_index)
{
   if (yaw_index % 4 == 0)
      return 0;
   else if (yaw_index % 2 == 0)
      return 2;
   else
      return 1;
}

bool collisionDetected(global float* height_map_params,
                       global float* planner_params,
                       global int* collision_offsets,
                       global float* height_map,
                       float2 point,
                       int yaw_index,
                       float height)
{
   int map_center_index = height_map_params[CENTER_INDEX];
   float map_resolution = height_map_params[HEIGHT_MAP_RESOLUTION];
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   int map_idx_x = coordinate_to_index(point.x, center.x, map_resolution, map_center_index);
   int map_idx_y = coordinate_to_index(point.y, center.y, map_resolution, map_center_index);
   float height_threshold = height + planner_params[GROUND_CLEARANCE];

   int offset_set = get_collision_set_index(yaw_index);

   int number_of_offsets = collision_offsets[offset_set];
   int x_offset_start = 3;
   int y_offset_start = x_offset_start + collision_offsets[0];
   for (int i = 0; i < offset_set; i++)
   {
      x_offset_start += 2 * collision_offsets[i];
      y_offset_start += collision_offsets[i] + collision_offsets[i + 1];
   }

   int cells_per_side = 2 * map_center_index + 1;

   for (int i = 0; i < number_of_offsets; i++)
   {
      int x_offset = collision_offsets[x_offset_start + i];
      int y_offset = collision_offsets[y_offset_start + i];

      int xQuery = map_idx_x + computeCollisionOffsetX(yaw_index, x_offset, y_offset);
      int yQuery = map_idx_y + computeCollisionOffsetY(yaw_index, x_offset, y_offset);

      if (xQuery < 0 || yQuery < 0 || xQuery >= cells_per_side || yQuery >= cells_per_side)
         continue;

      int map_key = indices_to_key(xQuery, yQuery, map_center_index);
      float heightQuery = height_map[map_key];

      if (isnan(heightQuery))
         continue;
      if (heightQuery >= height_threshold)
      {
         return true;
      }
   }

   return false;
}

void kernel computeHeuristicCost(global float* height_map_params, global float* planner_params, global float* heuristic_cost_map)
{
   int path_key = get_global_id(0);
   int path_center_index = (int) planner_params[PATH_CENTER_INDEX];
   float path_resolution = planner_params[PATH_RESOLUTION];

   int idx_x = key_to_x_index(path_key, path_center_index);
   int idx_y = key_to_y_index(path_key, path_center_index);
   int2 node_index = (int2) (idx_x, idx_y);

   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float2 node_position = indices_to_coordinate(node_index, center, path_resolution, path_center_index);
   float2 goal_position = (float2) (planner_params[GOAL_X], planner_params[GOAL_Y]);

   heuristic_cost_map[path_key] = length(goal_position - node_position);
}

void kernel computeEdgeData(global float* height_map_params,
                            global float* planner_params,
                            global int* neighbor_offsets,
                            global int* traversibility_offsets,
                            global int* collision_offsets,
                            global float* height_map,
                            global float* snapped_height_map,
                            global float* normal_least_squares_xyz_buffer,
                            global float* normal_ransac_xyz_buffer,
                            global int* edge_rejection_reason,
                            global float* delta_height_map,
                            global float* incline_map,
                            global float* roll_map,
                            global float* stance_traversibility_map,
                            global float* step_traversibility_map,
                            global float* incline_cost_map,
                            global float* roll_cost_map,
                            global float* traversibility_cost_map,
                            global float* edge_cost_map)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);
   int neighborIdx = get_global_id(2);

   int path_center_index = (int) planner_params[PATH_CENTER_INDEX];
   int map_center_index = (int) height_map_params[CENTER_INDEX];
   float map_resolution = height_map_params[HEIGHT_MAP_RESOLUTION];
   float path_resolution = planner_params[PATH_RESOLUTION];

   int path_key = indices_to_key(idx_x, idx_y, path_center_index);
   int2 node_index = (int2) (idx_x, idx_y);

   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float2 node_position = indices_to_coordinate(node_index, center, path_resolution, path_center_index);

   int number_of_neighbors = neighbor_offsets[0];

   // if the current snapped height is bad, invalidate all the edges and return
   float snapped_height = snapped_height_map[path_key];
   if (isnan(snapped_height))
   {
      // for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
      // {
      int edge_key = number_of_neighbors * path_key + neighborIdx;
      edge_rejection_reason[edge_key] = INVALID_SNAP;
      // }

      return;
   }

   int path_cells_per_side = 2 * path_center_index + 1;

   // for (int neighborIdx = 0; neighborIdx < number_of_neighbors; neighborIdx++)
   //{
   int neighbor_idx_x = idx_x + neighbor_offsets[1 + neighborIdx];
   int neighbor_idx_y = idx_y + neighbor_offsets[1 + number_of_neighbors + neighborIdx];
   int edge_key = number_of_neighbors * path_key + neighborIdx;

   if (neighbor_idx_x < 0 || neighbor_idx_x >= path_cells_per_side)
   {
      edge_rejection_reason[edge_key] = INVALID_SNAP;
      return;
   }
   if (neighbor_idx_y < 0 || neighbor_idx_y >= path_cells_per_side)
   {
      edge_rejection_reason[edge_key] = INVALID_SNAP;
      return;
   }

   int2 neighbor_index = (int2) (neighbor_idx_x, neighbor_idx_y);
   float2 neighbor_position = indices_to_coordinate(neighbor_index, center, path_resolution, path_center_index);
   int neighbor_path_key = indices_to_key(neighbor_idx_x, neighbor_idx_y, path_center_index);
   float snapped_neighbor_height = snapped_height_map[neighbor_path_key];

   if (isnan(snapped_neighbor_height))
   {
      edge_rejection_reason[edge_key] = INVALID_SNAP;
      return;
   }

   float xy_distance = length(neighbor_position - node_position);

   float delta_height = fabs(snapped_neighbor_height - snapped_height);
   float incline = atan2(delta_height, xy_distance);

   delta_height_map[edge_key] = delta_height;
   incline_map[edge_key] = incline;

   if (fabs(incline) > planner_params[MAX_INCLINE])
   {
      edge_rejection_reason[edge_key] = TOO_STEEP;
      return;
   }

   if (planner_params[CHECK_FOR_COLLISIONS] == 1.0F)
   {
      if (collisionDetected(height_map_params, planner_params, collision_offsets, height_map, neighbor_position, neighborIdx, snapped_neighbor_height))
      {
         edge_rejection_reason[edge_key] = COLLISION;
         return;
      }
   }

   float4 traversibility_measures = computeTraversibilityMeasures(height_map_params, planner_params, traversibility_offsets, snapped_height_map, height_map,
                                                                  normal_ransac_xyz_buffer, node_position, neighbor_position, neighborIdx);
   stance_traversibility_map[2 * edge_key] = traversibility_measures.s0;
   stance_traversibility_map[2 * edge_key + 1] = traversibility_measures.s1;
   step_traversibility_map[2 * edge_key] = traversibility_measures.s2;
   step_traversibility_map[2 * edge_key + 1] = traversibility_measures.s3;

   if (!computeIsTraversible(planner_params, traversibility_measures))
   {
      edge_rejection_reason[edge_key] = NON_TRAVERSIBLE;
      return;
   }

   // edge is valid, so set to -1
   edge_rejection_reason[edge_key] = -1;

   float edge_cost = xy_distance;
   float incline_cost = 0.0f;
   if (incline > planner_params[NOMINAL_INCLINE])
   {
      float inclineDelta = fabs((incline - planner_params[NOMINAL_INCLINE]));
      incline_cost = planner_params[INCLINE_COST_WEIGHT] * (max(0.0f, inclineDelta - planner_params[INCLINE_COST_DEADBAND]));
   }
   edge_cost += incline_cost;

   if (planner_params[COMPUTE_SURFACE_NORMALS] == 1.0f)
   {
      // compute roll cost
      float roll = computeRollAtNode(height_map_params, normal_least_squares_xyz_buffer, node_position, neighbor_position);
      float roll_cost = computeRollCost(planner_params, incline, roll);
      edge_cost += roll_cost;
      roll_map[edge_key] = roll;
      roll_cost_map[edge_key] = roll_cost;
   }
   else
   {
      roll_map[edge_key] = 0.0f;
      roll_cost_map[edge_key] = 0.0f;
   }

   if (planner_params[COMPUTE_SURFACE_NORMALS] == 1.0f && planner_params[COMPUTE_TRAVERSIBILITY])
   {
      // get the traversibility cost
      float traversibility_cost = computeTraversibilityCost(planner_params, traversibility_measures);
      edge_cost += traversibility_cost;
      traversibility_cost_map[edge_key] = traversibility_cost;
   }

   edge_cost_map[edge_key] = edge_cost;
   incline_cost_map[edge_key] = incline_cost;
   //}
}

float computeSmootherTraversibility(global float* height_map_params,
                                    global float* planner_params,
                                    global float* smoothing_params,
                                    global float* traversibility_offsets,
                                    global float* height_map_data,
                                    global float* normal_xyz_data,
                                    int side,
                                    float signY,
                                    float nominal_height,
                                    float2 waypoint_position,
                                    float waypoint_yaw)
{
   int number_of_sampled_cells = 0;

   float traversibility_score_numerator = 0.0f;
   float min_height = nominal_height - planner_params[TRAVERSIBILITY_HEIGHT_WINDOW];
   float max_height = nominal_height + planner_params[TRAVERSIBILITY_HEIGHT_WINDOW];

   int center_index = height_map_params[CENTER_INDEX];
   int cells_per_side = 2 * center_index + 1;
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   // we're using the negative value because we want to rotate it back
   float cYaw = cos(-waypoint_yaw);
   float sYaw = sin(-waypoint_yaw);

   int number_of_offsets = traversibility_offsets[0];
   for (int i = 0; i < number_of_offsets; i++)
   {
      float offset_x = traversibility_offsets[1 + i];
      float offset_y = traversibility_offsets[1 + number_of_offsets + i];
      if (side == 0)
         offset_y += planner_params[HALF_STANCE_WIDTH];
      else
         offset_y -= planner_params[HALF_STANCE_WIDTH];

      float2 local_offset = (float2) (offset_x, offset_y);
      float2 map_offset = apply2DRotationToVector2D(local_offset, cYaw, sYaw);

      float2 twiddled_position = waypoint_position + map_offset;
      int twiddled_x_index = coordinate_to_index(twiddled_position.x, center.x, resolution, center_index);
      int twiddled_y_index = coordinate_to_index(twiddled_position.y, center.y, resolution, center_index);

      if (twiddled_x_index < 0 || twiddled_y_index < 0 || twiddled_x_index >= cells_per_side || twiddled_y_index >= cells_per_side)
         continue;

      int twiddled_key = indices_to_key(twiddled_x_index, twiddled_y_index, center_index);
      float twiddled_height = height_map_data[twiddled_key];

      number_of_sampled_cells++;

      if (twiddled_height > min_height && twiddled_height < max_height)
      {
         if (epsilonEquals(twiddled_height, height_map_params[GROUND_HEIGHT_ESTIMATE], 1e-3))
         {
            traversibility_score_numerator += 1.0f;
         }
         else
         {
            // TODO extract magic number
            float non_ground_alpha;
            if (twiddled_height - height_map_params[GROUND_HEIGHT_ESTIMATE] < planner_params[TRAVERSIBILITY_HEIGHT_WINDOW])
            {
               non_ground_alpha = 0.6f;
            }
            else
            {
               non_ground_alpha = 1.0f;
            }

            float height_deadband = smoothing_params[TRAVERSIBILITY_SMOOTHING_HEIGHT_DEADBAND];
            float delta_height = max(0.0f, fabs(nominal_height - twiddled_height) - height_deadband);
            float cell_percentage = 1.0f - delta_height / planner_params[TRAVERSIBILITY_HEIGHT_WINDOW];

            float normal_z = normal_xyz_data[3 * twiddled_key + 2];
            float incline = acos(normal_z);
            float incline_alpha = (incline - planner_params[TRAVERSIBILITY_MIN_NORMAL_TO_PENALIZE]) /
                                  (planner_params[TRAVERSIBILITY_MAX_NORMAL_TO_PENALIZE] - planner_params[TRAVERSIBILITY_MIN_NORMAL_TO_PENALIZE]);
            incline_alpha = clamp(0.0f, 1.0f, incline_alpha);
            traversibility_score_numerator += cell_percentage * ((1.0f - planner_params[TRAVERSIBILITY_INCLINE_WEIGHT]) * non_ground_alpha +
                                                                 planner_params[TRAVERSIBILITY_INCLINE_WEIGHT] * incline_alpha);
         }
      }
   }

   if (number_of_sampled_cells < planner_params[MINIMUM_CELLS_FOR_TRAVERSIBLE])
   {
      return 0.0f;
   }
   else
   {
      return traversibility_score_numerator / number_of_sampled_cells;
   }
}

float shiftAngleInRange(float angleToShift, float angleStart)
{
   angleStart -= EPS_ANGLE_SHIFT;

   float TwoPi = 2.0f * M_PI_F;
   float deltaFromStart = fmod((angleToShift - angleStart), TwoPi);

   if (deltaFromStart < 0.0f)
      deltaFromStart += TwoPi;

   return angleStart + deltaFromStart;
}

float trimAngleMinusPiToPi(float angleToShift)
{
   return shiftAngleInRange(angleToShift, -M_PI_F);
}

float angleDifferenceMinusPiToPi(float angleA, float angleB)
{
   return trimAngleMinusPiToPi(angleA - angleB);
}

float computeDeltaHeadingMagnitude(float x0, float y0, float x1, float y1, float x2, float y2, float deadband)
{
   float heading0 = atan2(y1 - y0, x1 - x0);
   float heading1 = atan2(y2 - y1, x2 - x1);
   return max(fabs(angleDifferenceMinusPiToPi(heading1, heading0)) - deadband, 0.0f);
}

void kernel computeCurrentTraversibilityMap(global float* height_map_params,
                                            global float* planner_params,
                                            global float* smoother_params,
                                            global float* traversibility_nominal_offsets,
                                            global float* height_map_data,
                                            global float* snapped_node_height_data,
                                            global float* normal_xyz_data,
                                            global float* left_traversibility_map,
                                            global float* right_traversibility_map)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);
   int yaw_key = get_global_id(2);

   int center_index = height_map_params[CENTER_INDEX];
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   int map_key = indices_to_key(idx_x, idx_y, center_index);

   float2 waypoint_position = indices_to_coordinate((int2) (idx_x, idx_y), center, resolution, center_index);
   int path_key = coordinate_to_key(waypoint_position, center, planner_params[PATH_RESOLUTION], planner_params[PATH_CENTER_INDEX]);
   float waypoint_z = snapped_node_height_data[path_key];
   float waypoint_yaw = index_to_yaw(yaw_key, smoother_params[YAW_DISCRETIZATIONS]);

   float left_traversibility = computeSmootherTraversibility(height_map_params, planner_params, smoother_params, traversibility_nominal_offsets,
                                                             height_map_data, normal_xyz_data, 0, 1.0f, waypoint_z, waypoint_position, waypoint_yaw);
   float right_traversibility = computeSmootherTraversibility(height_map_params, planner_params, smoother_params, traversibility_nominal_offsets,
                                                              height_map_data, normal_xyz_data, 1, 1.0f, waypoint_z, waypoint_position, waypoint_yaw);

   int result_key = smoother_params[YAW_DISCRETIZATIONS] * map_key + yaw_key;
   left_traversibility_map[result_key] = left_traversibility;
   right_traversibility_map[result_key] = right_traversibility;
}

void kernel computeCollisionGradientMap(global float* height_map_params,
                                        global float* planner_params,
                                        global float* smoothing_params,
                                        global float* height_map_data,
                                        global float* snapped_height_map_data,
                                        global float* collision_gradients_map,
                                        global float* max_collision_map)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);
   int yaw_key = get_global_id(2);

   float box_size_x = smoothing_params[BOX_SIZE_X];
   float box_size_y = smoothing_params[BOX_SIZE_Y];
   float resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   int maxOffset = (int) round(0.5 * length((float2) (box_size_x, box_size_y)) / resolution);
   int center_index = height_map_params[CENTER_INDEX];

   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float2 waypoint_xy = indices_to_coordinate((int2) (idx_x, idx_y), center, resolution, center_index);
   float heading = index_to_yaw(yaw_key, smoothing_params[YAW_DISCRETIZATIONS]);

   int path_key = coordinate_to_key(waypoint_xy, center, planner_params[PATH_RESOLUTION], planner_params[PATH_CENTER_INDEX]);
   float waypointZ = snapped_height_map_data[path_key];

   float sH = sin(-heading);
   float cH = cos(-heading);

   float2 gradient = (float2) (0.0f, 0.0f);
   int numCollisions = 0;
   float height_threshold = waypointZ + smoothing_params[BOX_GROUND_OFFSET];
   float max_collision = 0.0f;

   float half_box_size_x = 0.5f * box_size_x;
   float half_box_size_y = 0.5f * box_size_y;

   int cells_per_axis = 2 * center_index + 1;

   for (int xi = -maxOffset; xi <= maxOffset; xi++)
   {
      for (int yi = -maxOffset; yi <= maxOffset; yi++)
      {
         int indexXi = idx_x + xi;
         int indexYi = idx_y + yi;

         if (indexXi < 0 || indexXi >= cells_per_axis || indexYi < 0 || indexYi >= cells_per_axis)
            continue;

         int2 indices = (int2) (indexXi, indexYi);
         float2 position = indices_to_coordinate(indices, center, resolution, center_index);

         float2 delta = position - waypoint_xy;

         // rotating this offset to the local frame
         // float2 delta_local = (float2) (cH * delta.x + sH * delta.y, -sH * delta.x + cH * delta.y);
         float2 delta_local = apply2DRotationToVector2D(delta, cH, sH);

         float absDyLocal = fabs(delta_local.y);

         if (fabs(delta_local.x) > half_box_size_x || absDyLocal > half_box_size_y)
            continue;

         int query_key = indices_to_key(indices.x, indices.y, center_index);
         float query_height = height_map_data[query_key];
         if (query_height < height_threshold)
            continue;

         float lateral_penetration = half_box_size_y - absDyLocal;
         max_collision = max(max_collision, lateral_penetration);

         float2 penetration = (float2) (0.0f, sign(delta_local.y) * lateral_penetration);
         gradient += apply2DRotationToVector2D(penetration, cH, sH);

         numCollisions++;
      }
   }

   if (numCollisions > 0)
      gradient = smoothing_params[COLLISION_WEIGHT] / numCollisions * gradient;

   int map_key = indices_to_key(idx_x, idx_y, center_index);
   int result_key = smoothing_params[YAW_DISCRETIZATIONS] * map_key + yaw_key;
   max_collision_map[result_key] = max_collision;
   collision_gradients_map[2 * result_key] = gradient.x;
   collision_gradients_map[2 * result_key + 1] = gradient.y;
}

void kernel computeTraversibilityForGradientMap(global float* height_map_params,
                                                global float* planner_params,
                                                global float* smoother_params,
                                                global float* traversibility_gradient_offsets,
                                                global float* height_map_data,
                                                global float* snapped_node_height_data,
                                                global float* normal_xyz_data,
                                                global float* left_traversibility_for_gradient_map,
                                                global float* right_traversibility_for_gradient_map)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);
   int yaw_key = get_global_id(2);

   int center_index = height_map_params[CENTER_INDEX];
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   float2 waypoint_position = indices_to_coordinate((int2) (idx_x, idx_y), center, resolution, center_index);
   float waypoint_yaw = index_to_yaw(yaw_key, smoother_params[YAW_DISCRETIZATIONS]);

   int path_key = coordinate_to_key(waypoint_position, center, planner_params[PATH_RESOLUTION], planner_params[PATH_CENTER_INDEX]);
   float waypoint_z = snapped_node_height_data[path_key];

   float left_neg_traversibility = computeSmootherTraversibility(height_map_params, planner_params, smoother_params, traversibility_gradient_offsets,
                                                                 height_map_data, normal_xyz_data, 0, -1.0f, waypoint_z, waypoint_position, waypoint_yaw);
   float left_pos_traversibility = computeSmootherTraversibility(height_map_params, planner_params, smoother_params, traversibility_gradient_offsets,
                                                                 height_map_data, normal_xyz_data, 0, 1.0f, waypoint_z, waypoint_position, waypoint_yaw);
   float right_neg_traversibility = computeSmootherTraversibility(height_map_params, planner_params, smoother_params, traversibility_gradient_offsets,
                                                                  height_map_data, normal_xyz_data, 1, -1.0f, waypoint_z, waypoint_position, waypoint_yaw);
   float right_pos_traversibility = computeSmootherTraversibility(height_map_params, planner_params, smoother_params, traversibility_gradient_offsets,
                                                                  height_map_data, normal_xyz_data, 1, 1.0f, waypoint_z, waypoint_position, waypoint_yaw);

   int map_key = indices_to_key(idx_x, idx_y, center_index);
   int result_key = smoother_params[YAW_DISCRETIZATIONS] * map_key + yaw_key;
   left_traversibility_for_gradient_map[2 * result_key] = left_neg_traversibility;
   left_traversibility_for_gradient_map[2 * result_key + 1] = left_pos_traversibility;
   right_traversibility_for_gradient_map[2 * result_key] = right_neg_traversibility;
   right_traversibility_for_gradient_map[2 * result_key + 1] = right_pos_traversibility;
}

void kernel computeGroundPlaneGradientMap(global float* height_map_params,
                                          global float* planner_params,
                                          global float* smoother_params,
                                          global int* ground_plane_offsets,
                                          global float* height_map_data,
                                          global int* left_cells_map,
                                          global int* right_cells_map,
                                          global float* gradient_map)
{
   int idx_x = get_global_id(0);
   int idx_y = get_global_id(1);
   int yaw_key = get_global_id(2);

   int center_index = height_map_params[CENTER_INDEX];
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   float resolution = height_map_params[HEIGHT_MAP_RESOLUTION];

   int map_key = indices_to_key(idx_x, idx_y, center_index);

   float2 waypoint_position = indices_to_coordinate((int2) (idx_x, idx_y), center, resolution, center_index);
   float waypoint_yaw = index_to_yaw(yaw_key, smoother_params[YAW_DISCRETIZATIONS]);

   float sH = sin(-waypoint_yaw);
   float cH = cos(-waypoint_yaw);

   int results_key = smoother_params[YAW_DISCRETIZATIONS] * map_key + yaw_key;

   int offsets = ground_plane_offsets[0];
   int cells_per_side = 2 * center_index + 1;

   for (int side = 0; side < 2; side++)
   {
      int ground_plane_cells = 0;

      for (int i = 0; i < offsets; i++)
      {
         float2 delta = (float2) (0.0f, 0.0f);
         delta.x = ground_plane_offsets[1 + i];
         delta.y = ground_plane_offsets[1 + offsets + i];
         if (side == 1)
            delta.y = -delta.y;
         delta *= height_map_params[HEIGHT_MAP_RESOLUTION];

         float2 delta_map = apply2DRotationToVector2D(delta, cH, sH);

         float2 query_position = waypoint_position + delta_map;
         int query_idx_x = coordinate_to_index(query_position.x, center.x, height_map_params[HEIGHT_MAP_RESOLUTION], center_index);
         int query_idx_y = coordinate_to_index(query_position.y, center.y, height_map_params[HEIGHT_MAP_RESOLUTION], center_index);

         if (query_idx_x < 0 || query_idx_x >= cells_per_side || query_idx_y < 0 || query_idx_y >= cells_per_side)
            continue;

         int query_key = indices_to_key(query_idx_x, query_idx_y, center_index);
         float query_z = height_map_data[query_key];

         bool isGroundPlane = fabs(query_z - height_map_params[GROUND_HEIGHT_ESTIMATE]) < 1e-3;
         if (isGroundPlane)
            ground_plane_cells++;
      }

      if (side == 0)
      {
         left_cells_map[results_key] = ground_plane_cells;
      }
      else
      {
         right_cells_map[results_key] = ground_plane_cells;
      }
   }

   int left_cells = left_cells_map[results_key];
   int right_cells = right_cells_map[results_key];

   float ground_plane_cell_count_delta = ((float) (left_cells - right_cells)) / offsets;
   float2 gradient_basis = (float2) (sH, cH);
   float2 gradient = ground_plane_cell_count_delta * smoother_params[FLAT_GROUND_WEIGHT] * gradient_basis;

   gradient_map[2 * results_key] = gradient.x;
   gradient_map[2 * results_key + 1] = gradient.y;
}

void kernel computeWaypointSmoothnessGradient(global float* smoothing_params,
                                              global float* waypoint_xyzYaw,
                                              global int* waypont_turn_points,
                                              global float* waypoint_smoothness_gradients)
{
   int waypoint_key = get_global_id(0);

   // return if it's the first or last gradient
   if (waypoint_key == 0 || waypoint_key == (smoothing_params[NUMBER_OF_WAYPOINTS] - 1))
      return;

   int isTurnPoint = waypont_turn_points[waypoint_key] == 1;

   float x0 = waypoint_xyzYaw[4 * (waypoint_key - 1)];
   float y0 = waypoint_xyzYaw[4 * (waypoint_key - 1) + 1];
   float x1 = waypoint_xyzYaw[4 * waypoint_key];
   float y1 = waypoint_xyzYaw[4 * waypoint_key + 1];
   float x2 = waypoint_xyzYaw[4 * (waypoint_key + 1)];
   float y2 = waypoint_xyzYaw[4 * (waypoint_key + 1) + 1];

   // Equal spacing gradient
   float spacingGradientX = -4.0f * smoothing_params[EQUAL_SPACING_WEIGHT] * (x2 - 2.0f * x1 + x0);
   float spacingGradientY = -4.0f * smoothing_params[EQUAL_SPACING_WEIGHT] * (y2 - 2.0f * y1 + y0);
   float alphaTurnPoint = isTurnPoint ? smoothing_params[TURN_POINT_SMOOTHNESS_DISCOUNT] : 1.0f;

   float2 gradient = alphaTurnPoint * (float2) (spacingGradientX, spacingGradientY);

   // Smoothness gradient
   if (!isTurnPoint)
   {
      float min_curvature_to_penalize = smoothing_params[MIN_CURVATURE_TO_PENALIZE];
      float gradient_epsilon = smoothing_params[GRADIENT_EPSILON];
      float smoothness_weight = smoothing_params[SMOOTHNESS_WEIGHT];
      float exp = 1.5f;
      double f0 = pow(computeDeltaHeadingMagnitude(x0, y0, x1, y1, x2, y2, min_curvature_to_penalize), exp);
      double fPdx = pow(computeDeltaHeadingMagnitude(x0, y0, x1 + gradient_epsilon, y1, x2, y2, min_curvature_to_penalize), exp);
      double fPdy = pow(computeDeltaHeadingMagnitude(x0, y0, x1, y1 + gradient_epsilon, x2, y2, min_curvature_to_penalize), exp);
      float smoothnessGradientX = smoothness_weight * (fPdx - f0) / gradient_epsilon;
      float smoothnessGradientY = smoothness_weight * (fPdy - f0) / gradient_epsilon;

      gradient.x += smoothnessGradientX;
      gradient.y += smoothnessGradientY;
   }
   waypoint_smoothness_gradients[2 * waypoint_key] = gradient.x;
   waypoint_smoothness_gradients[2 * waypoint_key + 1] = gradient.y;
}

void kernel getWaypointCurrentTraversibility(global float* height_map_params,
                                             global float* smoothing_params,
                                             global float* waypoint_xyzYaw,
                                             global float* left_traversibility_map,
                                             global float* right_traversibility_map,
                                             global float* waypoint_traversibility_values)
{
   int waypoint_key = get_global_id(0);

   // return if it's the first or last waypoint
   if (waypoint_key == 0 || waypoint_key == (smoothing_params[NUMBER_OF_WAYPOINTS] - 1))
      return;

   float2 waypoint_xy = (float2) (waypoint_xyzYaw[4 * waypoint_key], waypoint_xyzYaw[4 * waypoint_key + 1]);
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   int center_index = height_map_params[CENTER_INDEX];

   int map_key = coordinate_to_key(waypoint_xy, center, height_map_params[HEIGHT_MAP_RESOLUTION], center_index);
   int yaw_key = yaw_to_index(waypoint_xyzYaw[4 * waypoint_key + 3], smoothing_params[YAW_DISCRETIZATIONS]);

   int data_key = smoothing_params[YAW_DISCRETIZATIONS] * map_key + yaw_key;

   waypoint_traversibility_values[2 * waypoint_key] = left_traversibility_map[data_key];
   waypoint_traversibility_values[2 * waypoint_key + 1] = right_traversibility_map[data_key];
}

void kernel computeWaypointMapGradients(global float* height_map_params,
                                        global float* smoothing_params,
                                        global float* waypoint_xyzYaw,
                                        global int* waypont_turn_points,
                                        global float* collision_gradient_map,
                                        global int* max_collisions_map,
                                        global float* left_traversibilities_for_gradient_map,
                                        global float* right_traversibilities_for_gradient_map,
                                        global float* waypoint_traversibility_values,
                                        global int* left_ground_plane_cells_map,
                                        global int* right_ground_plane_cells_map,
                                        global float* ground_plane_gradient_map,
                                        global int* waypoint_max_collisions,
                                        global float* waypoint_collision_gradients,
                                        global float* waypoint_traversibility_samples,
                                        global float* waypoint_traversibility_gradients,
                                        global int* waypoint_ground_plane_cells,
                                        global float* waypoint_ground_plane_gradients)
{
   int waypoint_key = get_global_id(0);

   // return if it's the first or last gradient
   if (waypoint_key < 2 || waypoint_key > (smoothing_params[NUMBER_OF_WAYPOINTS] - 3))
      return;

   float2 waypoint_xy = (float2) (waypoint_xyzYaw[4 * waypoint_key], waypoint_xyzYaw[4 * waypoint_key + 1]);
   float waypoint_z = waypoint_xyzYaw[4 * waypoint_key + 2];
   float2 center = (float2) (height_map_params[centerX], height_map_params[centerY]);
   int center_index = height_map_params[CENTER_INDEX];

   int map_key = coordinate_to_key(waypoint_xy, center, height_map_params[HEIGHT_MAP_RESOLUTION], center_index);
   int yaw_key = yaw_to_index(waypoint_xyzYaw[4 * waypoint_key + 3], smoothing_params[YAW_DISCRETIZATIONS]);

   int data_key = smoothing_params[YAW_DISCRETIZATIONS] * map_key + yaw_key;

   // get the collision gradient
   waypoint_max_collisions[waypoint_key] = max_collisions_map[data_key];
   waypoint_collision_gradients[2 * waypoint_key] = collision_gradient_map[2 * data_key];
   waypoint_collision_gradients[2 * waypoint_key + 1] = collision_gradient_map[2 * data_key + 1];

   // get the traversibility gradient
   for (int side = 0; side <= 1; side++)
   {
      float local_traversibility_threshold = smoothing_params[SMOOTHER_TRAVERSIBILITY_THRESHOLD];

      float current_traversibility = waypoint_traversibility_values[2 * waypoint_key + side];
      float previous_traversibility_0 = waypoint_traversibility_values[2 * (waypoint_key - 1) + side];
      float previous_traversibility_1 = waypoint_traversibility_values[2 * (waypoint_key - 2) + side];
      float next_traversibility_0 = waypoint_traversibility_values[2 * (waypoint_key + 1) + side];
      float next_traversibility_1 = waypoint_traversibility_values[2 * (waypoint_key + 2) + side];
      float max_local_traversibility =
          max(max(max(max(current_traversibility, previous_traversibility_0), previous_traversibility_1), next_traversibility_0), next_traversibility_1);

      int goal_key = 4 * waypoint_key + 2 * side;

      if (max_local_traversibility > local_traversibility_threshold)
      {
         waypoint_traversibility_samples[goal_key] = 0.0f;
         waypoint_traversibility_samples[goal_key + 1] = 0.0f;
         waypoint_traversibility_gradients[goal_key] = 0.0f;
         waypoint_traversibility_gradients[goal_key + 1] = 0.0f;
      }
      else
      {
         global float* sided_buffer;
         if (side == 0)
            sided_buffer = left_traversibilities_for_gradient_map;
         else
            sided_buffer = right_traversibilities_for_gradient_map;

         float negative_sample = sided_buffer[2 * data_key];
         float positive_sample = sided_buffer[2 * data_key + 1];
         waypoint_traversibility_samples[goal_key] = negative_sample;
         waypoint_traversibility_samples[goal_key + 1] = positive_sample;

         float discount = (local_traversibility_threshold - max_local_traversibility) /
                          (local_traversibility_threshold - smoothing_params[TRAVERSIBILITY_THRESHOLD_FOR_NO_DISCOUNT]);
         discount = clamp(discount, 0.0f, 1.0f);

         float2 delta_local = (float2) (0.0f, discount * (positive_sample - negative_sample));
         float2 gradient = applyInverseYawRotationToVector2D(delta_local, waypoint_xyzYaw[4 * waypoint_key + 3]);
         ;

         waypoint_traversibility_gradients[goal_key] = gradient.x;
         waypoint_traversibility_gradients[goal_key + 1] = gradient.y;
      }
   }

   // get the ground plane gradient
   // TODO extract magic numbers
   float ground_plane_estimate = height_map_params[GROUND_HEIGHT_ESTIMATE];
   float height_threshold_for_ground = 0.015f;
   float current_height_above_ground_plane = waypoint_z - ground_plane_estimate;
   float next_height_above_ground_plane = waypoint_xyzYaw[4 * (waypoint_key + 1) + 3] - ground_plane_estimate;

   if (waypont_turn_points[waypoint_key] == 1 || current_height_above_ground_plane > height_threshold_for_ground ||
       next_height_above_ground_plane > height_threshold_for_ground)
   {
      waypoint_ground_plane_cells[2 * waypoint_key] = 0;
      waypoint_ground_plane_cells[2 * waypoint_key + 1] = 0;
      waypoint_ground_plane_gradients[2 * waypoint_key] = 0.0f;
      waypoint_ground_plane_gradients[2 * waypoint_key + 1] = 0.0f;
   }
   else
   {
      waypoint_ground_plane_cells[2 * waypoint_key] = left_ground_plane_cells_map[data_key];
      waypoint_ground_plane_cells[2 * waypoint_key + 1] = right_ground_plane_cells_map[data_key];
      waypoint_ground_plane_gradients[2 * waypoint_key] = ground_plane_gradient_map[2 * data_key];
      waypoint_ground_plane_gradients[2 * waypoint_key + 1] = ground_plane_gradient_map[2 * data_key + 1];
   }
}