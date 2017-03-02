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
      if (rrtTree.expandTree() == true)
      {
         for (int i = 0; i < rrtTree.newNode.getDimensionOfNodeData(); i++)
         {
            branchInfo[i] = rrtTree.newNode.getNodeData(i);
            branchInfo[i + rrtTree.newNode.getDimensionOfNodeData()] = rrtTree.nearNode.getNodeData(i);
         }

         if (rrtTree.newNode.getDistance(goalNode) < rrtTree.getStepLength())
         {
            rrtTree.newNode.addChildNode(goalNode);
            rrtTree.updatePath(goalNode);
            optimalPath = rrtTree.pathNode;

            return true;
         }
         return false;
      }
      return false;
   }
}
