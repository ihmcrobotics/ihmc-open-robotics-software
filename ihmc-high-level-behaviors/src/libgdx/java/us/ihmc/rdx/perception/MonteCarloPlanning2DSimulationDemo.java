package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.behaviors.monteCarloPlanning.Agent;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanner;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlannerTools;
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
      planner.getWorld().submitObstacles(obstacles);

      int screenSize = 1400;
      boolean running = true;
      int i = 0;
      while (running)
      {
         planner.scanWorld();

         Mat gridColor = new Mat();
         MonteCarloPlannerTools.plotWorldAndAgent(planner, gridColor);

         PerceptionDebugTools.display("Grid", gridColor, 1, screenSize);

         Point2D newState = planner.plan();
         planner.execute(newState);

         i += 1;
      }
   }
}
