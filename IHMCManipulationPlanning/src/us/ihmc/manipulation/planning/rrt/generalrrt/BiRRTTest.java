package us.ihmc.manipulation.planning.rrt.generalrrt;

import us.ihmc.commons.PrintTools;

public class BiRRTTest
{

   
   public static void main(String[] args)
   {
      PrintTools.info("Start");
      
      RRTNode startNode = new RRTNode2D(0.0, 0.0);
      RRTNode goalNode = new RRTNode2D(3.0, 2.0);
      RRTPlanner2D rrtPlanner = new RRTPlanner2D(startNode, goalNode, 0.6);

      RRTNode upperBoundNode = new RRTNode2D(5.0, 5.0);
      RRTNode lowerBoundNode = new RRTNode2D(-5.0, -5.0);
      rrtPlanner.getRRTTree().setUpperBound(upperBoundNode);
      rrtPlanner.getRRTTree().setLowerBound(lowerBoundNode);

      int maxNumberOfExpanding = 1500;

      if (rrtPlanner.expandTreeGoal(maxNumberOfExpanding) == true)
      {
         rrtPlanner.updateOptimalPath(101, 100);   
      }
      else
      {
         PrintTools.info("Fail");
      }
      
      JPanelDrawer drawer = new JPanelDrawer();
      
      drawer.xBoundary = new double[] {lowerBoundNode.getNodeData(0), upperBoundNode.getNodeData(0)};
      drawer.yBoundary = new double[] {lowerBoundNode.getNodeData(1), upperBoundNode.getNodeData(1)};
            
      drawer.validNodes = rrtPlanner.getRRTTree().getWholeNodes();
      drawer.invalidNodes = rrtPlanner.getRRTTree().getFailNodes();
      drawer.pathNodes = rrtPlanner.getRRTTree().getPathNodes();
      drawer.optimalNodes = rrtPlanner.getOptimalPath();
      drawer.draw();
      
      PrintTools.info("End");
   }
}
