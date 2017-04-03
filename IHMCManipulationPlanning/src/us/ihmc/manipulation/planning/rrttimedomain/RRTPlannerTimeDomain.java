package us.ihmc.manipulation.planning.rrttimedomain;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;

public class RRTPlannerTimeDomain
{
   private RRTNode rootNode;

   private RRTTreeTimeDomain rrtTree;

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
               return true;
            }
         }
      }
      return false;
   }
   
   public RRTTreeTimeDomain getTree()
   {
      return rrtTree;
   }
   
   
}
