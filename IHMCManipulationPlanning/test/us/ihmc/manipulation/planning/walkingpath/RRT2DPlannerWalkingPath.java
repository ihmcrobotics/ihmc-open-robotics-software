package us.ihmc.manipulation.planning.walkingpath;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTPlanner;

public class RRT2DPlannerWalkingPath extends RRTPlanner
{

   public RRT2DPlannerWalkingPath(RRTNode root, RRTNode goal, double stepLength)
   {
      super(root, goal, stepLength);
   }

   public boolean expandTreeGoal(RRTNode nodeOne, RRTNode nodeTwo)
   {
      double branchRootData[] = new double[2];
      double branchNewData[] = new double[2];
      
      if (rrtTree.expandTree() == true)
      {
         for (int i = 0; i < rrtTree.getNewNode().getDimensionOfNodeData(); i++)
         {
            branchRootData[i] = rrtTree.getNewNode().getNodeData(i);
            branchNewData[i] = rrtTree.getNearNode().getNodeData(i);
            nodeOne.setNodeData(i, rrtTree.getNewNode().getNodeData(i));
            nodeTwo.setNodeData(i, rrtTree.getNearNode().getNodeData(i));
         }

         if (rrtTree.getNewNode().getDistance(goalNode) < rrtTree.getStepLength())
         {
            rrtTree.getNewNode().addChildNode(goalNode);
            rrtTree.updatePath(goalNode);
            optimalPath = rrtTree.getPathNode();

            return true;
         }
         return false;
      }
      return false;
   }
}
