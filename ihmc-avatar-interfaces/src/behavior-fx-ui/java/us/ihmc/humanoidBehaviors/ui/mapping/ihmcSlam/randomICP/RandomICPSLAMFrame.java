package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.randomICP;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.AbstractSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;

public class RandomICPSLAMFrame extends SLAMFrame
{
   public RandomICPSLAMFrame(StereoVisionPointCloudMessage message)
   {
      super(message);
   }

   public RandomICPSLAMFrame(SLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      super(frame, message);
   }

   @Override
   public SLAMFrameOptimizerCostFunction createCostFunction(AbstractSLAM slam)
   {
      RandomICPSLAMFrameOptimizerCostFunction function = new RandomICPSLAMFrameOptimizerCostFunction(getInitialSensorPoseToWorld(),
                                                                                                     getOriginalPointCloudToSensorPose(), previousFrame,
                                                                                                     slam.getOctreeResolution());
      return function;
   }
}
