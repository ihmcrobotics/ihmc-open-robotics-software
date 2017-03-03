package us.ihmc.manipulation.planning.rrt;

/**
 * @method expandTreeGoal re-factor a double array to save every node data of the tree. 
 * 
 * @author InhoLee 170224
 *
 */

public class RRT2DPlanner extends RRTPlanner
{
   public RRT2DPlanner(RRTNode root, RRTNode goal, double stepLength)
   {
      super(root, goal, stepLength);
   }

   public boolean expandTreeGoal(double[] branchInfo)
   {
      if (getRRTTree().expandTree() == true)
      {
         for (int i = 0; i < getRRTTree().newNode.getDimensionOfNodeData(); i++)
         {
            branchInfo[i] = getRRTTree().newNode.getNodeData(i);
            branchInfo[i + getRRTTree().newNode.getDimensionOfNodeData()] = getRRTTree().nearNode.getNodeData(i);
         }

         if (getRRTTree().newNode.getDistance(getGoalNode()) < getRRTTree().getStepLength())
         {
            getRRTTree().newNode.addChildNode(getGoalNode());
            getRRTTree().updatePath(getGoalNode());
            setOptimalPath(getRRTTree().pathNode);

            return true;
         }
         return false;
      }
      return false;
   }
}
