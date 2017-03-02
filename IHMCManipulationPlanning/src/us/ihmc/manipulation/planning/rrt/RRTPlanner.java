package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;
/**
 * This class provides only expanding tree, expanding tree until reaching goal node and updating an optimal path.
 * The optimal path is just displacement minimizing straight-line shortcut.
 *  
 * @author InhoLee 170224
 *
 */
public class RRTPlanner
{
   public RRTNode rootNode;
   public RRTNode goalNode;

   public RRTTree rrtTree;
   RRTPiecewisePath rrtPiecewisePath;
   public ArrayList<RRTNode> optimalPath = new ArrayList<RRTNode>();   
   

   public RRTPlanner(RRTNode root, RRTNode goal, double stepLength)
   {
      this.rootNode = root;
      this.goalNode = goal;

      rrtTree = new RRTTree(this.rootNode);
      rrtTree.setStepLength(stepLength);
   }
   
   public void expandTreeWhole(int numberOfExpanding)
   {
      for(int i =0;i<numberOfExpanding;i++)
      {
         rrtTree.expandTree();         
      }
   }

   public boolean expandTreeGoal(int maxNumberOfExpanding)
   {
      for(int i=0;i<maxNumberOfExpanding;i++)
      {
         if(rrtTree.expandTree() == true)
         {            
            if(rrtTree.newNode.getDistance(goalNode) < rrtTree.getStepLength())
            {
               rrtTree.newNode.addChildNode(goalNode);            
               rrtTree.updatePath(goalNode);
               optimalPath = rrtTree.pathNode;
               
               return true;
            }  
         }                 
      }  
      return false;
   }
   
   public void updateOptimalPath(int sizeOfPiecewisePath)
   {
      rrtPiecewisePath = new RRTPiecewisePath(optimalPath, sizeOfPiecewisePath);
      
      if(rrtPiecewisePath.updateShortCutPath() == true)
      {
         optimalPath = rrtPiecewisePath.shortcutPath;
      }
   }
   
   public void updateOptimalPath(int sizeOfPiecewisePath, int numberOfIteration)
   {
      for (int i =0;i<numberOfIteration;i++)
      {
         updateOptimalPath(sizeOfPiecewisePath);
      }
   }

}
