package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTPiecewisePath;
import us.ihmc.manipulation.planning.rrt.RRTTree;
import us.ihmc.manipulation.planning.rrt.RRTValidConnection;

public class RRTPlannerTimeDomain
{
   protected RRTNode rootNode;
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
               PrintTools.info("Reach "+i);
               rrtTree.updatePathNode(rrtTree.getNewNode());
               optimalPath = rrtTree.getPathNode();
               return true;
            }
         }
      }
      return false;
   }
   
   public void updateShortCut(int sizeOfPiecewisePath)
   {
      if (optimalPath.size() > 1)
      {
         rrtPiecewisePath = new RRTPiecewisePath(optimalPath, sizeOfPiecewisePath);

         if (rrtPiecewisePath.updateShortCutPath() == true)
         {
            optimalPath = rrtPiecewisePath.getShortCutPath();
         }
      }
      else
      {
      }
   }

   public void updateOptimalPath(int sizeOfPiecewisePath, int numberOfIteration)
   {
      PrintTools.info("Buliding Started");      
      if(firstShortCut(sizeOfPiecewisePath) == true)
      {
         
      }
      else
      {
         for (int i = 0; i < numberOfIteration; i++)
         {
            updateShortCut(sizeOfPiecewisePath);
            PrintTools.info("Try Shortcut "+i);
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
      }

      PrintTools.info("OptimalPath is Built");
   }
   
   public void updateOptimalPath(int numberOfIteration)
   {
      int sizeOfPiecewisePath = optimalPath.size() * 4;
      
      PrintTools.info("Buliding Started");
      if(firstShortCut(sizeOfPiecewisePath) == true)
      {
         PrintTools.info("First cut Success");
      }
      else
      {
         for (int i = 0; i < numberOfIteration; i++)
         {
            updateShortCut(sizeOfPiecewisePath);
            PrintTools.info("Try Shortcut "+i);
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
      }
      
      
      PrintTools.info("OptimalPath is Built");
   }
   
   private boolean firstShortCut(int numberOfPiece)
   {
      RRTValidConnection fastShortCut = new RRTValidConnection(optimalPath.get(0), optimalPath.get(optimalPath.size()-1));
      fastShortCut.reInitialize(numberOfPiece);
      if(fastShortCut.isValidConnection() == true)
      {
         ArrayList<RRTNode> tempPath = new ArrayList<RRTNode>();
         tempPath.add(optimalPath.get(0));
         tempPath.add(optimalPath.get(optimalPath.size()-1));

         optimalPath = tempPath;
         return true;
      }
      else
      {
         return false;         
      }
   }
   
   public RRTTreeTimeDomain getTree()
   {
      return rrtTree;
   }
   
   public ArrayList<RRTNode> getOptimalPath()
   {
      return optimalPath;
   }
   
   public ArrayList<RRTNode> getFailNodes()
   {
      return rrtTree.failNodes;
   } 
   
}
