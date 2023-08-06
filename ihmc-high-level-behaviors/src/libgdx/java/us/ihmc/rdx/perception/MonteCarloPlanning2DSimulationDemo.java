package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.behaviors.monteCarloPlanning.Agent;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanner;
import us.ihmc.behaviors.monteCarloPlanning.World;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.util.ArrayList;

import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_GRAY2RGB;

class MonteCarloPlanning2DSimulationDemo
{
   public static void main(String[] args)
   {
      ArrayList<Vector4D32> obstacles = new ArrayList<>();

      obstacles.add(new Vector4D32(10, 20, 5, 5));
      obstacles.add(new Vector4D32(45, 61, 8, 8));
      obstacles.add(new Vector4D32(70, 82, 10, 8));
      obstacles.add(new Vector4D32(90, 43, 3, 3));
      obstacles.add(new Vector4D32(20, 34, 6, 6));

      //obstacles.add(new Vector4D32(30, 10, 1, 10));
      //obstacles.add(new Vector4D32(30, 28, 1, 2));
      //obstacles.add(new Vector4D32(15, 30, 15, 1));
      //obstacles.add(new Vector4D32(60, 30, 1, 30));
      //obstacles.add(new Vector4D32(10, 60, 10, 1));
      //obstacles.add(new Vector4D32(58, 60, 2, 1));
      //obstacles.add(new Vector4D32(80, 40, 1, 40));
      //obstacles.add(new Vector4D32(30, 80, 30, 1));
      //obstacles.add(new Vector4D32(75, 80, 5, 1));

      MonteCarloPlanner planner = new MonteCarloPlanner();
      planner.addObstacles(obstacles);

      int screenSize = 1400;
      boolean running = true;
      int i = 0;
      while (running)
      {
         planner.collectObservations();

         plotWorldCV(planner.getWorld(), planner.getAgent(), screenSize);

         Point2D newState = planner.plan();

         i += 1;
      }
   }

   public static void plotWorldCV(World world, Agent agent, int screenSize)
   {
      // Convert the floating point grid to 8-bit grayscale then convert it to RGB image
      Mat gridColor = new Mat();
      opencv_imgproc.cvtColor(world.getGrid(), gridColor, COLOR_GRAY2RGB);

      for (Vector4D32 obstacle : world.getObstacles())
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
                 new Scalar(150, 150, 150, 150), -1, 0, 0
         );
      }

      // Display a yellow square on goal of total width goal_margin
      int goalMargin = world.getGoalMargin();
      int goalMinX = (int) (world.getGoal().getX32() - goalMargin);
      int goalMaxX = (int) (world.getGoal().getX32() + goalMargin);
      int goalMinY = (int) (world.getGoal().getY32() - goalMargin);
      int goalMaxY = (int) (world.getGoal().getY32() + goalMargin);

      opencv_imgproc.rectangle(gridColor,
              new Point(goalMinY, goalMinX),
              new Point(goalMaxY, goalMaxX),
              new Scalar(255, 255, 255, 255), -1, 0, 0
      );

      // Set the agent's position as 50
      gridColor.ptr((int)(agent.getPreviousPosition().getX32()), (int)(agent.getPreviousPosition().getY32())).put(new byte[] {0, 0, 0});
      gridColor.ptr((int)(agent.getPosition().getX32()), (int)(agent.getPosition().getY32())).put(new byte[] {0, (byte) 255, (byte) 250});

      gridColor.ptr((int)(agent.getAveragePosition().getX32()), (int)(agent.getAveragePosition().getY32())).put(new byte[] {100, 100, (byte) 255});

      // Plot lidar scan points as filled red cells
      for (Point2D point : agent.getScanPoints())
      {
         if (point.getX32() < gridColor.rows() && point.getY32() < gridColor.cols() && point.getX32() >= 0 && point.getY32() >= 0)
         {
            gridColor.ptr((int) point.getX32(), (int) point.getY32()).put(new byte[] {0, 0, (byte) 255});
         }
      }

      PerceptionDebugTools.display("Grid", gridColor, 50, screenSize);
   }
}
