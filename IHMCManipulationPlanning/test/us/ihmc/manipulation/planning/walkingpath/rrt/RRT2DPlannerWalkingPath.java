package us.ihmc.manipulation.planning.walkingpath.rrt;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTPlanner;
import us.ihmc.manipulation.planning.walkingpath.footstep.SkeletonPathFootStepPlanner;
import us.ihmc.tools.io.printing.PrintTools;

public class RRT2DPlannerWalkingPath extends RRTPlanner
{
   SkeletonPathFootStepPlanner footStepPlanner;

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
            
            PrintTools.info("path size is "+getOptimalPath().size());
            return true;
         }
         return false;
      }
      return false;
   }
   
   public void createFootStepPlanner(double stepLength, double stepWidth)
   {
      double[] pathX = new double[getOptimalPath().size()];
      double[] pathY = new double[getOptimalPath().size()];
      
      for(int i=0;i<getOptimalPath().size();i++)
      {
         pathX[i] = getOptimalPath().get(i).getNodeData(0);
         pathY[i] = getOptimalPath().get(i).getNodeData(1);
      }
      footStepPlanner = new SkeletonPathFootStepPlanner(pathX, pathY, stepLength, stepWidth);
   }
   
   public SkeletonPathFootStepPlanner getFootStepPlanner()
   {
      return footStepPlanner;
   }
}
