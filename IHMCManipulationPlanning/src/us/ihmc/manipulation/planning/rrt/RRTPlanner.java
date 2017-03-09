package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;

import us.ihmc.tools.io.printing.PrintTools;

/**
 * This class provides only expanding tree, expanding tree until reaching goal node and updating an optimal path.
 * The optimal path is just displacement minimizing straight-line shortcut.
 *  
 * @author InhoLee 170224
 *
 */
public class RRTPlanner
{
   private RRTNode rootNode;
   private RRTNode goalNode;

   private RRTTree rrtTree;
   private RRTPiecewisePath rrtPiecewisePath;
   private ArrayList<RRTNode> optimalPath = new ArrayList<RRTNode>();

   public RRTPlanner(RRTNode root, RRTNode goal, double stepLength)
   {
      this.rootNode = root;
      this.goalNode = goal;

      rrtTree = new RRTTree(this.rootNode);
      rrtTree.setStepLength(stepLength);
   }

   public void expandTreeWhole(int numberOfExpanding)
   {
      for (int i = 0; i < numberOfExpanding; i++)
      {
         rrtTree.expandTree();
      }
   }

   public boolean expandTreeGoal(int maxNumberOfExpanding)
   {
      for (int i = 0; i < maxNumberOfExpanding; i++)
      {
         if (rrtTree.expandTree() == true)
         {
            if (rrtTree.newNode.getDistance(goalNode) < rrtTree.getStepLength())
            {
               rrtTree.newNode.addChildNode(goalNode);
               rrtTree.updatePath(goalNode);
               optimalPath = rrtTree.pathNode;

               PrintTools.info("Connected " + i);

               return true;
            }
         }
      }
      return false;
   }

   public void updateOptimalPath(int sizeOfPiecewisePath)
   {
      if (optimalPath.size() > 1)
      {
         rrtPiecewisePath = new RRTPiecewisePath(optimalPath, sizeOfPiecewisePath);

         if (rrtPiecewisePath.updateShortCutPath() == true)
         {
            optimalPath = rrtPiecewisePath.shortcutPath;
         }
      }
      else
      {
         //PrintTools.info("There is no path");
      }
   }

   public void updateOptimalPath(int sizeOfPiecewisePath, int numberOfIteration)
   {
      PrintTools.info("Buliding Started");
      for (int i = 0; i < numberOfIteration; i++)
      {
         updateOptimalPath(sizeOfPiecewisePath);
      }

      if (optimalPath.size() > 2)
      {
         ArrayList<RRTNode> tempPath = new ArrayList<RRTNode>();
         tempPath.add(optimalPath.get(0));
         for (int i = 1; i < optimalPath.size(); i++)
         {
            if (optimalPath.get(0).getDistance(optimalPath.get(i)) > 0)
            {
               tempPath.add(optimalPath.get(i));
            }
         }
         optimalPath = tempPath;
      }

      PrintTools.info("OptimalPath is Built");
   }

   public RRTTree getRRTTree()
   {
      return rrtTree;
   }

   public RRTPiecewisePath getPiecewisePath()
   {
      return rrtPiecewisePath;
   }

   public ArrayList<RRTNode> getOptimalPath()
   {
      return optimalPath;
   }

   public void setOptimalPath(ArrayList<RRTNode> path)
   {
      optimalPath = path;
   }

   public RRTNode getGoalNode()
   {
      return goalNode;
   }
}
