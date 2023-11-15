package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.*;

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
      LogTools.info(String.format("ID: %d\tLevel: %d\tNode: %s\tChildren: %d%n",
               node.getId(),
               level,
               node.getPosition().toString(),
               node.getChildren().size()));

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
    * */
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
      for (int i = 0; i<neighbors.length; i++)
         neighbors[i] = new Point2D();

      for (int i = 0; i<samples; i++)
      {
         double alpha = i / (double) samples;
         currentPoint.setToZero();
         currentPoint.interpolate(startPoint, endPoint, alpha);

         updateNeighborsList(neighbors, currentPoint);

         // check if a 3x3 square around the current point is occupied
         if ( isPointOccupied(currentPoint, grid) || Arrays.stream(neighbors).anyMatch(neighbor -> isPointOccupied(neighbor, grid)))
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

   public static void plotPath(List<MonteCarloTreeNode> path, Mat gridColor)
   {
      for (MonteCarloTreeNode node : path)
      {
         MonteCarloWaypointNode waypointNode = (MonteCarloWaypointNode) node;
         Point2DReadOnly position = waypointNode.getPosition();

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

   public static FootstepPlan getFootstepPlanFromTree(MonteCarloFootstepNode root)
   {
      List<MonteCarloTreeNode> path = new ArrayList<>();
      MonteCarloPlannerTools.getOptimalPath(root, path);

      FootstepPlan footstepPlan = new FootstepPlan();
      for (MonteCarloTreeNode node : path)
      {
         MonteCarloFootstepNode footstepNode = (MonteCarloFootstepNode) node;
         Point3D position = footstepNode.getPosition();
         position.scale(1/50.0f);
         FramePose3D footstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(position), new Quaternion(position.getZ(), 0, 0));
         footstepPlan.addFootstep(footstepNode.getRobotSide(), footstepPose);

         LogTools.info("Footstep Side: {}, Position: {}", footstepNode.getRobotSide(), position);
      }
      return footstepPlan;
   }

   public static void getFootstepActionGrid(ArrayList<Vector3D> actions, Point3D origin, int side)
   {
      actions.clear();
      for (int i = 20; i <= 40; i+=2)
      {
         for (int j = 20; j <= 40; j+=2)
         {
            actions.add(new Vector3D(i * side, j * side, 0));
         }
      }
   }

   public static MonteCarloTreeNode getBestNode(MonteCarloFootstepNode root)
   {
      float bestScore = 0;
      MonteCarloTreeNode bestNode = null;

      for (MonteCarloTreeNode node : root.getChildren())
      {
         node.updateUpperConfidenceBound();
         if (node.getUpperConfidenceBound() > bestScore)
         {
            bestScore = node.getUpperConfidenceBound();
            bestNode = node;
         }
      }

      return bestNode;
   }

   public static void plotFootstepNodeList(List<MonteCarloFootstepNode> nodes, Mat gridColor)
   {
      for (MonteCarloFootstepNode node : nodes)
      {
         Point3D position = node.getPosition();
         position.add((double) gridColor.cols() / 2, (double) gridColor.rows() / 2, 0);

         double score = node.getValue();
         if (isWithinGridBoundaries(new Point2D(position), gridColor.cols()) && node.getRobotSide() == RobotSide.LEFT)
         {
            gridColor.ptr((int) position.getX32(), (int) position.getY32()).put(new byte[] {0, (byte) (score * 255), (byte) (score * 255), (byte) 255});
         }
         else if (isWithinGridBoundaries(new Point2D(position), gridColor.cols()) && node.getRobotSide() == RobotSide.RIGHT)
         {
            gridColor.ptr((int) position.getX32(), (int) position.getY32()).put(new byte[] {(byte) (score * 255), (byte) (score * 255), (byte) 255, (byte) 255});
         }
      }
   }

   public static double scoreFootstepNode(MonteCarloFootstepNode oldNode, MonteCarloFootstepNode newNode, MonteCarloFootstepPlannerRequest request)
   {
      //(randomState.getPosition().distanceSquared(request.getGoalFootPoses().get(RobotSide.LEFT).getPosition()) < world.getGoalMarginSquared())

      double edgeCost = Math.abs(0.5f - oldNode.getPosition().distance(newNode.getPosition())) * 0.01f;

      int rIndex = (int) (newNode.getPosition().getX() + request.getContactMap().rows() / 2);
      int cIndex = (int) (newNode.getPosition().getY() + request.getContactMap().cols() / 2);

      Point3D goalPosition = request.getGoalFootPoses().get(RobotSide.LEFT).getPosition();
      goalPosition.scale(50.0f);
      goalPosition.add((double) request.getContactMap().rows() / 2, (double) request.getContactMap().cols() / 2, 0);

      double score = ((int) request.getContactMap().ptr(rIndex, cIndex).get() & 0xFF) / 255.0;
      score += 1.0f / newNode.getPosition().distance(goalPosition);
      //score -= edgeCost;
      return score;
   }
}
