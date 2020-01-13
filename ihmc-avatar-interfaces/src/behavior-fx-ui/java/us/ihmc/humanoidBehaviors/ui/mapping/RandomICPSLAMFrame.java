package us.ihmc.humanoidBehaviors.ui.mapping;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;

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
