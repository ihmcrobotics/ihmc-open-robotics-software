package us.ihmc.manipulation.planning.rrttimedomain;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTPiecewisePath;
import us.ihmc.manipulation.planning.rrt.RRTTree;

public class RRTPlannerTimeDomain
{
   private RRTNode rootNode;
   private RRTTreeTimeDomain rrtTree;

   private RRTPiecewisePath rrtPiecewisePath;
   private ArrayList<RRTNode> optimalPath = new ArrayList<RRTNode>();
   
   public RRTPlannerTimeDomain(RRTNode root)
   {
      this.rootNode = root;

      rrtTree = new RRTTreeTimeDomain(this.rootNode);      
   }

   public void expandTreeWhole(int numberOfExpanding)
   {
      for (int i = 0; i < numberOfExpanding; i++)
      {
         rrtTree.expandTreeTimeDomain();
      }
   }
   
   public boolean expandTreeGoal(int maxNumberOfExpanding)
   {
      for (int i = 0; i < maxNumberOfExpanding; i++)
      {
         if (rrtTree.expandTreeTimeDomain() == true)
         {
            if (rrtTree.getTime(rrtTree.getNewNode()) == rrtTree.getMotionTime())
            {
               PrintTools.info("Reach ");
               rrtTree.updatePath(rrtTree.getNewNode());
               optimalPath = rrtTree.getPathNode();
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
            optimalPath = rrtPiecewisePath.getShortCutPath();
            PrintTools.info("Shortcut");
         }
      }
      else
      {
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
   
   public RRTTreeTimeDomain getTree()
   {
      return rrtTree;
   }
   
   public ArrayList<RRTNode> getOptimalPath()
   {
      return optimalPath;
   }
   
   
}
