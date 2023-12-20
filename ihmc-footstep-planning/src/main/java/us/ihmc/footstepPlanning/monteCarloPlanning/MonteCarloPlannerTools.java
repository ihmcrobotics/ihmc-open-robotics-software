package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_GRAY2RGB;

public class MonteCarloPlannerTools
{
   public static int getTotalNodesInSubTree(MonteCarloTreeNode rootNode)
   {
      if (rootNode == null)
         return 0;

      int total = 1;
      for (MonteCarloTreeNode child : rootNode.getChildren())
      {
         total += getTotalNodesInSubTree(child);
      }

      return total;
   }

   public static void printTree(MonteCarloWaypointNode node, int level)
   {
      // TODO: Make this write to a string and then print the combined string with LogTools.info()
      LogTools.info(String.format("ID: %d\tLevel: %d\tNode: %s\tChildren: %d%n", node.getId(), level, node.getState().toString(), node.getChildren().size()));

      for (MonteCarloTreeNode child : node.getChildren())
      {
         printTree((MonteCarloWaypointNode) child, level + 1);
      }
   }

   public static void plotWorld(MonteCarloPlanningWorld world, Mat gridColor)
   {
      // Convert the floating point grid to 8-bit grayscale then convert it to RGB image
      opencv_imgproc.cvtColor(world.getGrid(), gridColor, COLOR_GRAY2RGB);
   }

   public static void plotRangeScan(ArrayList<Point2DReadOnly> scanPoints, Mat gridColor)
   {
      // Plot lidar scan points as filled red cells
      for (Point2DReadOnly point : scanPoints)
      {
         // check bounds of point
         if (isWithinGridBoundaries(point, gridColor.cols()))
         {
            gridColor.ptr((int) point.getX32(), (int) point.getY32()).put(new byte[] {0, 0, (byte) 255});
         }
      }
   }

   public static void plotAgent(MonteCarloWaypointAgent agent, Mat gridColor)
   {
      // check bounds of agent
      if (isWithinGridBoundaries(agent.getState(), gridColor.cols()))
      {
         // Set the agent's position as 50
         gridColor.ptr((int) (agent.getPreviousPosition().getX32()), (int) (agent.getPreviousPosition().getY32())).put(new byte[] {0, 0, 0});
         gridColor.ptr((int) (agent.getState().getX32()), (int) (agent.getState().getY32())).put(new byte[] {0, (byte) 255, (byte) 250});
         gridColor.ptr((int) (agent.getAveragePosition().getX32()), (int) (agent.getAveragePosition().getY32())).put(new byte[] {100, 100, (byte) 255});
      }
   }

   public static void plotGoal(Point2DReadOnly goal, int goalMargin, Mat gridColor)
   {
      // Display a yellow square on goal of total width goal_margin
      int goalMinX = (int) (goal.getX32() - goalMargin);
      int goalMaxX = (int) (goal.getX32() + goalMargin);
      int goalMinY = (int) (goal.getY32() - goalMargin);
      int goalMaxY = (int) (goal.getY32() + goalMargin);

      opencv_imgproc.rectangle(gridColor, new Point(goalMinY, goalMinX), new Point(goalMaxY, goalMaxX), new Scalar(255, 255, 255, 255), -1, 0, 0);
   }

   /**
    * Fills the obstacles in the grid given a list of rectangular obstacles
    * in the form (center_x, center_y, width, height) using OpenCV rectangular drawing function.
    */
   public static void fillObstacles(ArrayList<Vector4D32> obstacles, Mat gridColor)
   {
      for (Vector4D32 obstacle : obstacles)
      {
         int obstacleSizeX = (int) obstacle.getZ32();
         int obstacleSizeY = (int) obstacle.getS32();
         int obstacleMinX = (int) (obstacle.getX32() - obstacleSizeX);
         int obstacleMaxX = (int) (obstacle.getX32() + obstacleSizeX);
         int obstacleMinY = (int) (obstacle.getY32() - obstacleSizeY);
         int obstacleMaxY = (int) (obstacle.getY32() + obstacleSizeY);

         // Draw a red rectangle on the obstacle
         opencv_imgproc.rectangle(gridColor,
                                  new Point(obstacleMinY, obstacleMinX),
                                  new Point(obstacleMaxY, obstacleMaxX),
                                  new Scalar(MonteCarloPlannerConstants.OCCUPIED),
                                  -1,
                                  0,
                                  0);
      }
   }

   public static boolean isPointOccupied(Point2DReadOnly point, Mat grid)
   {
      return grid.ptr((int) point.getX(), (int) point.getY()).get() == MonteCarloPlannerConstants.OCCUPIED;
   }

   public static Point2DReadOnly findClosestOccupiedPoint(Point2DReadOnly startPoint, Point2DReadOnly endPoint, Mat grid, float maxRange)
   {
      // set closestPoint to max, max
      Point2D closestPoint = new Point2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

      int samples = 10;
      Point2D currentPoint = new Point2D();

      Point2D[] neighbors = new Point2D[8];
      for (int i = 0; i < neighbors.length; i++)
         neighbors[i] = new Point2D();

      for (int i = 0; i < samples; i++)
      {
         double alpha = i / (double) samples;
         currentPoint.setToZero();
         currentPoint.interpolate(startPoint, endPoint, alpha);

         updateNeighborsList(neighbors, currentPoint);

         // check if a 3x3 square around the current point is occupied
         if (isPointOccupied(currentPoint, grid) || Arrays.stream(neighbors).anyMatch(neighbor -> isPointOccupied(neighbor, grid)))
         {
            // If the current point is occupied, return the closest point
            closestPoint.set(currentPoint);
            break;
         }
      }

      if (closestPoint.distance(startPoint) > maxRange)
      {
         closestPoint.set(endPoint);
      }

      return closestPoint;
   }

   public static void updateNeighborsList(Point2D[] neighborsToUpdate, Point2DReadOnly origin)
   {
      neighborsToUpdate[0].set(origin.getX() + 1, origin.getY());
      neighborsToUpdate[1].set(origin.getX() - 1, origin.getY());
      neighborsToUpdate[2].set(origin.getX(), origin.getY() + 1);
      neighborsToUpdate[3].set(origin.getX(), origin.getY() - 1);
      neighborsToUpdate[4].set(origin.getX() + 1, origin.getY() + 1);
      neighborsToUpdate[5].set(origin.getX() - 1, origin.getY() - 1);
      neighborsToUpdate[6].set(origin.getX() + 1, origin.getY() - 1);
      neighborsToUpdate[7].set(origin.getX() - 1, origin.getY() + 1);
   }

   public static void updateGrid(MonteCarloPlanningWorld world, Point2DReadOnly agentState, int radius)
   {
      // set a circle of pixels around the agent to be 50
      int agentMinX = (int) (agentState.getX() - radius);
      int agentMaxX = (int) (agentState.getX() + radius);
      int agentMinY = (int) (agentState.getY() - radius);
      int agentMaxY = (int) (agentState.getY() + radius);

      int radiusSquared = radius * radius;

      for (int x = agentMinX; x < agentMaxX; x++)
      {
         for (int y = agentMinY; y < agentMaxY; y++)
         {
            // if point is within 5 pixels circular radius
            if ((MathTools.square(x - agentState.getX()) + MathTools.square(y - agentState.getY()) < radiusSquared))
            //if (Math.sqrt(Math.pow(x - agentState.getX(), 2) + Math.pow(y - agentState.getY(), 2)) < radius)
            {
               // check if inside the world boundaries
               if (x >= 0 && x <= world.getGridWidth() && y >= 0 && y <= world.getGridHeight())
               {
                  if (world.getGrid().ptr(x, y).get() == MonteCarloPlannerConstants.OCCUPANCY_UNKNOWN)
                  {
                     world.getGrid().ptr(x, y).put(MonteCarloPlannerConstants.OCCUPANCY_FREE);
                  }
               }
            }
         }
      }
   }

   public static boolean isWithinGridBoundaries(Point2DReadOnly position, int gridWidth)
   {
      return position.getX() >= 0 && position.getX() < gridWidth && position.getY() >= 0 && position.getY() < gridWidth;
   }

   public static void getOptimalPath(MonteCarloTreeNode root, List<MonteCarloTreeNode> path)
   {
      float maxValue = -1e10f;
      MonteCarloTreeNode maxNode = null;
      for (MonteCarloTreeNode node : root.getChildren())
      {
         if (node.getValue() > maxValue)
         {
            maxValue = node.getValue();
            maxNode = node;
         }
      }

      if (maxNode != null)
      {
         path.add(maxNode);
         getOptimalPath(maxNode, path);
      }
   }

   public static void getOptimalPathByVisits(MonteCarloTreeNode root, List<MonteCarloTreeNode> path)
   {
      float maxVisits = -1e10f;
      MonteCarloTreeNode maxNode = null;
      for (MonteCarloTreeNode node : root.getChildren())
      {
         if (node.getVisits() > maxVisits)
         {
            maxVisits = node.getVisits();
            maxNode = node;
         }
      }

      if (maxNode != null)
      {
         path.add(maxNode);
         getOptimalPathByVisits(maxNode, path);
      }
   }

   public static void plotPath(List<MonteCarloTreeNode> path, Mat gridColor)
   {
      for (MonteCarloTreeNode node : path)
      {
         MonteCarloWaypointNode waypointNode = (MonteCarloWaypointNode) node;
         Point2DReadOnly position = waypointNode.getState();

         // check bounds of the grid
         if (isWithinGridBoundaries(position, gridColor.cols()))
            gridColor.ptr((int) position.getX32(), (int) position.getY32()).put((byte) 255, (byte) 100, (byte) 255);
      }
   }

   public static void getLayerCounts(MonteCarloTreeNode root, HashMap<Integer, Integer> layerCounts)
   {
      if (layerCounts.get(root.getLevel()) == null)
         layerCounts.put(root.getLevel(), 1);
      else
         layerCounts.merge(root.getLevel(), 1, Integer::sum);

      for (MonteCarloTreeNode child : root.getChildren())
      {
         getLayerCounts(child, layerCounts);
      }
   }

   public static void printLayerCounts(MonteCarloTreeNode root)
   {
      HashMap<Integer, Integer> layerCounts = new HashMap<>();
      MonteCarloPlannerTools.getLayerCounts(root, layerCounts);

      StringBuilder output = new StringBuilder("{");
      for (Integer key : layerCounts.keySet())
      {
         output.append("(").append(key - root.getLevel()).append(":").append(layerCounts.get(key)).append(")");
         output.append(", ");
      }

      LogTools.info("Layer Counts: {}", output.toString());
   }

   public static String getLayerCountsString(MonteCarloTreeNode root)
   {
      HashMap<Integer, Integer> layerCounts = new HashMap<>();
      MonteCarloPlannerTools.getLayerCounts(root, layerCounts);

      StringBuilder output = new StringBuilder("{");
      for (Integer key : layerCounts.keySet())
      {
         output.append("(").append(key - root.getLevel()).append(":").append(layerCounts.get(key)).append(")");
         output.append(", ");
      }

      return output.toString();
   }

   public static FootstepPlan getFootstepPlanFromTree(MonteCarloFootstepNode root, MonteCarloFootstepPlannerRequest request)
   {
      List<MonteCarloTreeNode> path = new ArrayList<>();
      MonteCarloPlannerTools.getOptimalPath(root, path);
      LogTools.info("Optimal Path Size: {}", path.size());

      FootstepPlan footstepPlan = new FootstepPlan();
      for (MonteCarloTreeNode node : path)
      {
         MonteCarloFootstepNode footstepNode = (MonteCarloFootstepNode) node;

         float nodeX = footstepNode.getState().getX32() / 50.0f;
         float nodeY = footstepNode.getState().getY32() / 50.0f;
         float nodeZ = request.getTerrainMapData().getHeightInWorld(nodeX, nodeY);
         float nodeYaw = footstepNode.getState().getZ32();

         FramePose3D footstepPose = getFramePose3D(nodeX, nodeY, nodeZ, nodeYaw);
         footstepPlan.addFootstep(footstepNode.getRobotSide(), footstepPose);

         LogTools.debug("Footstep Node -> Position: {}, Yaw: {}", footstepPose.getPosition(), footstepPose.getYaw());
      }
      return footstepPlan;
   }

   private static FramePose3D getFramePose3D(double xPosition, double yPosition, float zPosition, double yaw)
   {
      Point3D position = new Point3D(xPosition, yPosition, zPosition);
      Quaternion orientation = new Quaternion(yaw, 0, 0);
      return new FramePose3D(ReferenceFrame.getWorldFrame(), position, orientation);
   }

   public static void getFootstepActionGrid(ArrayList<Vector3D> actions, int side)
   {
      actions.clear();
      for (int i = 10; i <= 30; i += 4)
      {
         for (int j = 10; j <= 30; j += 4)
         {
            actions.add(new Vector3D(i, j * side, 0));
         }
      }
   }

   public static void getFootstepRadialActionSet(MonteCarloFootstepPlannerParameters parameters, ArrayList<Vector3D> actions, float yawPrevious, int side)
   {
      actions.clear();

      float sidedYawOffset = side * (float) parameters.getSidedYawOffset();
      float adjustedPreviousYaw = yawPrevious + sidedYawOffset;
      float yawMin = adjustedPreviousYaw - (float) parameters.getSearchYawBand();
      float yawMax = adjustedPreviousYaw + (float) parameters.getSearchYawBand();
      float minRadius = (float) parameters.getSearchInnerRadius() * 50.0f;
      float maxRadius = (float) parameters.getSearchOuterRadius() * 50.0f;
      for (int i = -30; i <= 30; i += parameters.getSearchSkipSize())
      {
         for (int j = -30; j <= 30; j += parameters.getSearchSkipSize())
         {
            float radius = (float) Math.sqrt(i * i + j * j);
            float yaw = (float) Math.atan2(j, i);

            //LogTools.info(String.format("(%d, %d) Radius: %.2f (%.2f, %.2f), Yaw: %.2f (%.2f, %.2f)", i, j, radius,
            //                            parameters.getSearchInnerRadius(), parameters.getSearchOuterRadius(), yaw, yawMin, yawMax));

            if (radius >= minRadius && radius <= maxRadius && yaw >= yawMin && yaw <= yawMax)
            {
               actions.add(new Vector3D(i, j, -0.1));
               actions.add(new Vector3D(i, j, 0));
               actions.add(new Vector3D(i, j, 0.1));
            }
         }
      }
   }

   public static MonteCarloTreeNode getBestNode(MonteCarloFootstepNode root)
   {
      float bestScore = 0;
      MonteCarloTreeNode bestNode = null;

      for (MonteCarloTreeNode node : root.getChildren())
      {
         if (node.getValue() > bestScore)
         {
            bestScore = node.getValue();
            bestNode = node;
         }
      }

      return bestNode;
   }

   public static void plotFootstepNodeList(List<MonteCarloFootstepNode> nodes, Mat gridColor)
   {
      for (MonteCarloFootstepNode node : nodes)
      {
         Point3D position = node.getState();
         position.add((double) gridColor.cols() / 2, (double) gridColor.rows() / 2, 0);

         double score = node.getValue();
         if (isWithinGridBoundaries(new Point2D(position), gridColor.cols()) && node.getRobotSide() == RobotSide.LEFT)
         {
            gridColor.ptr((int) position.getX32(), (int) position.getY32()).put(new byte[] {0, (byte) (score * 255), (byte) (score * 255), (byte) 255});
         }
         else if (isWithinGridBoundaries(new Point2D(position), gridColor.cols()) && node.getRobotSide() == RobotSide.RIGHT)
         {
            gridColor.ptr((int) position.getX32(), (int) position.getY32())
                     .put(new byte[] {(byte) (score * 255), (byte) (score * 255), (byte) 255, (byte) 255});
         }
      }
   }

   public static double scoreFootstepNode(MonteCarloFootstepNode oldNode,
                                          MonteCarloFootstepNode newNode,
                                          MonteCarloFootstepPlannerRequest request,
                                          MonteCarloFootstepPlannerParameters plannerParameters,
                                          boolean debug)
   {
      double score = 0.0;

      int offsetX = (int) (request.getTerrainMapData().getSensorOrigin().getX() * 50);
      int offsetY = (int) (request.getTerrainMapData().getSensorOrigin().getY() * 50);

      int rIndex = (int) (newNode.getState().getX() + request.getTerrainMapData().getLocalGridSize() / 2) - offsetX;
      int cIndex = (int) (newNode.getState().getY() + request.getTerrainMapData().getLocalGridSize() / 2) - offsetY;

      Point2D previousPosition = new Point2D(oldNode.getState());
      Point2D currentPosition = new Point2D(newNode.getState());
      Point2D goalPositionRight = new Point2D(request.getGoalFootPoses().get(RobotSide.LEFT).getPosition());
      Point2D goalPositionLeft = new Point2D(request.getGoalFootPoses().get(RobotSide.RIGHT).getPosition());
      Point2D goalPosition = new Point2D();
      goalPosition.add(goalPositionRight, goalPositionLeft);
      goalPosition.scale(0.5f);
      goalPosition.scale(50.0f);

      Point2D startPositionLeft = new Point2D(request.getStartFootPoses().get(RobotSide.LEFT).getPosition());
      Point2D startPositionRight = new Point2D(request.getStartFootPoses().get(RobotSide.RIGHT).getPosition());
      Point2D startPosition = new Point2D();
      startPosition.add(startPositionLeft, startPositionRight);
      startPosition.scale(0.5f);
      startPosition.scale(50.0f);

      Vector2D stepVector = new Vector2D();
      stepVector.sub(currentPosition, previousPosition);

      Vector2D goalVector = new Vector2D();
      goalVector.sub(goalPosition, previousPosition);
      goalVector.normalize();

      double progressToGoal = goalVector.dot(stepVector);

      double yawFromStartToGoal = Math.atan2(goalPosition.getY() - startPosition.getY(), goalPosition.getX() - startPosition.getX());
      double yawDifferenceFromReference = Math.abs(yawFromStartToGoal - oldNode.getState().getZ());
      double distanceFromReferenceLine = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentPosition, startPosition, goalPosition);
      double referenceCost = distanceFromReferenceLine * 10.0f + yawDifferenceFromReference * 10.0f;

      double goalReward = plannerParameters.getGoalReward() * progressToGoal;
      double contactReward =
            (((int) request.getTerrainMapData().getContactScoreLocal(rIndex, cIndex) & 0xFF) / 255.0 - plannerParameters.getFeasibleContactCutoff())
            * plannerParameters.getFeasibleContactReward();

      double stepYawCost = Math.abs(oldNode.getState().getZ() - newNode.getState().getZ()) * 0.01f;
      double stepDistanceCost = Math.abs(previousPosition.distance(currentPosition)) * 0.01f;
      double stepHeightCost = (request.getTerrainMapData().getHeightLocal(rIndex, cIndex) - request.getTerrainMapData().getHeightLocal(rIndex, cIndex)) * 0.01f;
      double edgeCost = stepYawCost + stepDistanceCost + stepHeightCost;

      if (debug)
         LogTools.info(String.format("Rewards -> Goal: %.2f, Contact: %.2f, Edge: %.2f, Reference: %.2f", goalReward, contactReward, edgeCost, referenceCost));

      score = goalReward + contactReward - edgeCost - referenceCost;
      return score;
   }

   public static MonteCarloTreeNode getOptimalNode(MonteCarloTreeNode root, int layer)
   {
      if (root.getLevel() == layer - 1)
      {
         return root.getMaxQueueNode();
      }
      else
      {
         return getOptimalNode(root.getMaxQueueNode(), layer);
      }
   }

   public static void resetNodeVisits(MonteCarloTreeNode root)
   {
      root.setVisits(0);
      for (MonteCarloTreeNode child : root.getChildren())
      {
         resetNodeVisits(child);
      }
   }

   public static void resetNodeLevels(MonteCarloTreeNode root, int level)
   {
      root.setLevel(0);
      for (MonteCarloTreeNode child : root.getChildren())
      {
         resetNodeLevels(child, level + 1);
      }
   }

   public static void pruneTree(MonteCarloTreeNode root, int numberOfChildrenToKeep)
   {
      for (MonteCarloTreeNode child : root.getChildren())
      {
         pruneTree(child, numberOfChildrenToKeep);
      }
      root.sortChildren();
      root.prune(numberOfChildrenToKeep);
   }
}
