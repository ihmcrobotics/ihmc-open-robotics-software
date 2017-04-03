package us.ihmc.manipulation.planning.rrttimedomain;

import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelPoseValidityTester;
import us.ihmc.manipulation.planning.rrt.RRTNode;

public class RRT1DNodeTimeDomain extends RRTNode
{
   public static SolarPanelPoseValidityTester nodeValidityTester;

   /*
    * getNodeData(0) = time getNodeData(1) = pelvisYaw
    */

   public RRT1DNodeTimeDomain()
   {
      super(2);
   }

   public RRT1DNodeTimeDomain(double timeK, double pelvisYawK)
   {
      super(2);
      super.setNodeData(0, timeK);
      super.setNodeData(1, pelvisYawK);
   }   
   
   @Override
   public boolean isValidNode()
   {
      nodeValidityTester.setSolarPanelWholeBodyPose(getNodeData(0), getNodeData(1));

      return nodeValidityTester.isValid;
   }

   @Override
   public RRTNode createNode()
   {
      return new RRT1DNodeTimeDomain();
   }
}
