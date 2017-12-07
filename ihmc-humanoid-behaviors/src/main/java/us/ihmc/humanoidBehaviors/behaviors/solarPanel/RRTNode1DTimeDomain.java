package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class RRTNode1DTimeDomain extends RRTNode
{
   public static SolarPanelPoseValidityTester nodeValidityTester;
   public static SolarPanelPath cleaningPath;
   
   /*
    * getNodeData(0) = time getNodeData(1) = pelvisYaw
    */

   public RRTNode1DTimeDomain()
   {
      super(2);
   }

   public RRTNode1DTimeDomain(double timeK, double pelvisYawK)
   {
      super(2);
      super.setNodeData(0, timeK);
      super.setNodeData(1, pelvisYawK);
   }  
   
   @Override
   public boolean isValidNode()
   {
      nodeValidityTester.setWholeBodyPose(cleaningPath, getNodeData(0), getNodeData(1));
      
      return nodeValidityTester.isValid();
   }

   @Override
   public RRTNode createNode()
   {
      return new RRTNode1DTimeDomain();
   }

   @Override
   public void setRandomNodeData()
   {
      
   }
}
