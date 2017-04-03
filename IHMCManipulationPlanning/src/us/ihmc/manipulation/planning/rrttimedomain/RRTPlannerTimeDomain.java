package us.ihmc.manipulation.planning.rrttimedomain;

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
   
   public RRTTreeTimeDomain getTree()
   {
      return rrtTree;
   }
   
   
}
