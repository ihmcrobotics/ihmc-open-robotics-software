package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.WheneverWholeBodyPoseTester;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.tools.thread.ThreadTools;

public class RRTNode3DTimeDomain extends RRTNode
{
   public static WheneverWholeBodyPoseTester nodeValidityTester;
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
      nodeValidityTester.onBehaviorEntered();
      nodeValidityTester.setWholeBodyPose(cleaningPath, getNodeData(0), getNodeData(1), getNodeData(2), getNodeData(3));
      
      nodeValidityTester.setUpHasBeenDone();
      
      while(true)
      {         
         if(nodeValidityTester.isDone())
         {
            PrintTools.info("try break");
            break;
         }  
         else
         {
            ThreadTools.sleep(50);
            PrintTools.info("not yet");
         }
            
      }
      PrintTools.info("return " + nodeValidityTester.isValid());
      
      return nodeValidityTester.isValid();
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
