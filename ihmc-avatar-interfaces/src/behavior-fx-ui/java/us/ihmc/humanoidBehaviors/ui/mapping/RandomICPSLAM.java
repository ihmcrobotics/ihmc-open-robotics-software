package us.ihmc.humanoidBehaviors.ui.mapping;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class RandomICPSLAM extends AbstractSLAM<RandomICPSLAMFrame>
{
   public static final double OCTREE_RESOLUTION = 0.02;

   public RandomICPSLAM(boolean doNaiveSLAM)
   {
      super(doNaiveSLAM, OCTREE_RESOLUTION);
   }

   @Override
   public RigidBodyTransform computeOptimizedMultiplier(SLAMFrame newFrame)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public SLAMFrame createFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      return new RandomICPSLAMFrame(pointCloudMessage);
   }

   @Override
   public SLAMFrame createFrame(SLAMFrame previousFrame, StereoVisionPointCloudMessage pointCloudMessage)
   {
      return new RandomICPSLAMFrame(previousFrame, pointCloudMessage);
   }
}