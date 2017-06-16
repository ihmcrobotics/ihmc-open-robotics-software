package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.WheneverWholeBodyValidityTester;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.tools.thread.ThreadTools;

public class RRTNode3DTimeDomain extends RRTNode
{
   public static WheneverWholeBodyValidityTester nodeValidityTester;
   public static SolarPanelPath cleaningPath;
   
   public RRTNode3DTimeDomain()
   {
      super(4);
   }

   public RRTNode3DTimeDomain(double timeK, double pelvisHeight, double chestYaw, double chestPitch)
   {
      super(4);
      super.setNodeData(0, timeK);
      super.setNodeData(1, pelvisHeight);
      super.setNodeData(2, chestYaw);
      super.setNodeData(3, chestPitch);
   }  
   
   @Override
   public boolean isValidNode()
   {
      PrintTools.info("isvalid START");
      
      nodeValidityTester.initialize();
      
      for(int i=0;i<50;i++)
      {
         PrintTools.info(""+i);
         nodeValidityTester.updateInternal();
         ThreadTools.sleep(10);
      }
      
      PrintTools.info("isvalid END");
      return true;
   }

   @Override
   public RRTNode createNode()
   {
      return new RRTNode3DTimeDomain();
   }

   @Override
   public void setRandomNodeData()
   {
      // TODO Auto-generated method stub
      
   }
}
