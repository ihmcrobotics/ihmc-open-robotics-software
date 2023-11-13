package us.ihmc.rdx.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPathPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloTreeNode;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloWaypointNode;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.util.ArrayList;
import java.util.List;

class MonteCarloPlanning2DSimulationDemo
{
   public static void main(String[] args)
   {
      ArrayList<Vector4D32> obstacles = new ArrayList<>(); // The obstacles here are represented as rectangles with (center_x, center_y, width, height)

      obstacles.add(new Vector4D32(20, 40, 10, 10));
      obstacles.add(new Vector4D32(90, 120, 16, 16));
      obstacles.add(new Vector4D32(140, 164, 10, 8));
      obstacles.add(new Vector4D32(180, 86, 6, 6));
      obstacles.add(new Vector4D32(40, 68, 12, 12));

      //obstacles.add(new Vector4D32(30, 10, 1, 10));
      //obstacles.add(new Vector4D32(30, 28, 1, 2));
      //obstacles.add(new Vector4D32(15, 30, 15, 1));
      //obstacles.add(new Vector4D32(60, 30, 1, 30));
      //obstacles.add(new Vector4D32(10, 60, 10, 1));
      //obstacles.add(new Vector4D32(58, 60, 2, 1));
      //obstacles.add(new Vector4D32(80, 40, 1, 40));
      //obstacles.add(new Vector4D32(30, 80, 30, 1));
      //obstacles.add(new Vector4D32(75, 80, 5, 1));

      MonteCarloPathPlanner planner = new MonteCarloPathPlanner(0);
      planner.getWorld().submitObstacles(obstacles);

      int screenSize = 1400;
      boolean running = true;
      int i = 0;
      Mat gridColor = new Mat();
      Scalar zero = new Scalar(0, 0, 0, 0);

      List<MonteCarloTreeNode> optimalPath = new ArrayList<>();

      planner.setParameters(50, 10, 4);

      while (running)
      {
         planner.scanWorld();

         gridColor.put(zero);
         MonteCarloPlannerTools.plotWorld(planner.getWorld(), gridColor);
         MonteCarloPlannerTools.plotAgent(planner.getAgent(), gridColor);
         MonteCarloPlannerTools.plotRangeScan(planner.getAgent().getScanPoints(), gridColor);
         MonteCarloPlannerTools.plotGoal(planner.getWorld().getGoal(), planner.getWorld().getGoalMargin(), gridColor);

         optimalPath.clear();
         planner.getOptimalPathFromRoot(optimalPath);
         MonteCarloPlannerTools.plotPath(optimalPath, gridColor);

         int totalNodes = planner.getNumberOfNodesInTree();
         
         //LogTools.info("Optimal Path Length: {}/({})", optimalPath.size(), totalNodes);

         MonteCarloPlannerTools.printLayerCounts(planner.getRoot());

         PerceptionDebugTools.display("Grid", gridColor, 1, screenSize);

         MonteCarloWaypointNode newState = (MonteCarloWaypointNode) planner.plan();
         planner.updateState(newState);

         i += 1;
      }
   }
}
