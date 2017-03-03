package us.ihmc.manipulation.planning.walkingpath;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTPlanner;
import us.ihmc.tools.io.printing.PrintTools;

public class RRT2DPlannerWalkingPath extends RRTPlanner
{

   public RRT2DPlannerWalkingPath(RRTNode root, RRTNode goal, double stepLength)
   {
      super(root, goal, stepLength);
   }

   public boolean expandTreeGoal(RRTNode nodeOne, RRTNode nodeTwo)
   {
      if (getRRTTree().expandTree() == true)
      {
         for (int i = 0; i < getRRTTree().getNewNode().getDimensionOfNodeData(); i++)
         {
            nodeOne.setNodeData(i, getRRTTree().getNewNode().getNodeData(i));
            nodeTwo.setNodeData(i, getRRTTree().getNearNode().getNodeData(i));
         }

         if (getRRTTree().getNewNode().getDistance(getGoalNode()) < getRRTTree().getStepLength())
         {
            getRRTTree().getNewNode().addChildNode(getGoalNode());
            getRRTTree().updatePath(getGoalNode());
            setOptimalPath(getRRTTree().getPathNode());
            
            PrintTools.info("111 path size is "+getOptimalPath().size());
            return true;
         }
         return false;
      }
      return false;
   }
}
