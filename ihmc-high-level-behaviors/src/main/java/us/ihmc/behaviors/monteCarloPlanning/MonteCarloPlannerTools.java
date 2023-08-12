package us.ihmc.behaviors.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Vector4D32;

import java.util.ArrayList;

import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_GRAY2RGB;

public class MonteCarloPlannerTools
{
   public static int getTotalNodes(MonteCarloTreeNode node)
   {
      if (node == null)
         return 0;

      int total = 1;
      for (MonteCarloTreeNode child : node.getChildren())
      {
         total += getTotalNodes(child);
      }

      return total;
   }

   public static void printTree(MonteCarloTreeNode node, int level)
   {
      System.out.printf("ID: %d\tLevel: %d\tNode: %s\tChildren: %d%n",
                        node.getId(),
                        level,
                        node.getAgentState().getPosition().toString(),
                        node.getChildren().size());

      for (MonteCarloTreeNode child : node.getChildren())
      {
         printTree(child, level + 1);
      }
   }

   public static void plotWorld(World world, Mat gridColor)
   {
      // Convert the floating point grid to 8-bit grayscale then convert it to RGB image
      opencv_imgproc.cvtColor(world.getGrid(), gridColor, COLOR_GRAY2RGB);
   }

   public static void plotRangeScan(ArrayList<Point2D> scanPoints, Mat gridColor)
   {
      // Plot lidar scan points as filled red cells
      for (Point2D point : scanPoints)
      {
         if (point.getX32() < gridColor.rows() && point.getY32() < gridColor.cols() && point.getX32() >= 0 && point.getY32() >= 0)
         {
            gridColor.ptr((int) point.getX32(), (int) point.getY32()).put(new byte[] {0, 0, (byte) 255});
         }
      }
   }

   public static void plotAgent(Agent agent, Mat gridColor)
   {
      // Set the agent's position as 50
      gridColor.ptr((int) (agent.getPreviousPosition().getX32()), (int) (agent.getPreviousPosition().getY32()))
               .put(new byte[] {0, 0, 0});
      gridColor.ptr((int) (agent.getPosition().getX32()), (int) (agent.getPosition().getY32()))
               .put(new byte[] {0, (byte) 255, (byte) 250});
      gridColor.ptr((int) (agent.getAveragePosition().getX32()), (int) (agent.getAveragePosition().getY32()))
               .put(new byte[] {100, 100, (byte) 255});
   }

   public static void plotGoal(Point2D goal, int goalMargin, Mat gridColor)
   {
      // Display a yellow square on goal of total width goal_margin
      int goalMinX = (int) (goal.getX32() - goalMargin);
      int goalMaxX = (int) (goal.getX32() + goalMargin);
      int goalMinY = (int) (goal.getY32() - goalMargin);
      int goalMaxY = (int) (goal.getY32() + goalMargin);

      opencv_imgproc.rectangle(gridColor, new Point(goalMinY, goalMinX), new Point(goalMaxY, goalMaxX), new Scalar(255, 255, 255, 255), -1, 0, 0);
   }

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
                                  new Scalar(100),
                                  -1,
                                  0,
                                  0);
      }
   }


   public static boolean isPointOccupied(Point2D point, Mat grid)
   {
      if (grid.ptr((int) point.getX(), (int) point.getY()).get() == 100)
         return true;
      else
         return false;
   }

   public static Point2D findClosestIntersection(Point2D start_point, Point2D end_point, Mat grid)
   {

      // set closest_point to max, max
      Point2D closest_point = new Point2D(10000000, 10000000);

      int samples = 10;
      for (int i = 0; i<samples; i++)
      {
         Point2D currentPoint = new Point2D(EuclidCoreTools.interpolate(start_point.getX(), end_point.getX(), i/(double)samples),
                                            EuclidCoreTools.interpolate(start_point.getY(), end_point.getY(), i/(double)samples));

         // check if a 3x3 square around the current point is occupied
         if (isPointOccupied(currentPoint, grid) ||
             isPointOccupied(new Point2D(currentPoint.getX() + 1, currentPoint.getY()), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX() - 1, currentPoint.getY()), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX(), currentPoint.getY() + 1), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX(), currentPoint.getY() - 1), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX() + 1, currentPoint.getY() + 1), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX() - 1, currentPoint.getY() - 1), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX() + 1, currentPoint.getY() - 1), grid) ||
             isPointOccupied(new Point2D(currentPoint.getX() - 1, currentPoint.getY() + 1), grid))
         {
            // If the current point is occupied, return the closest point
            return currentPoint;
         }
      }

      return closest_point;
   }
}
