package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FramePose;

public class ConstantConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose configuration;

   public ConstantConfigurationProvider(ReferenceFrame referenceFrame)
   {
      this.configuration = new FramePose(referenceFrame);
   }

   public ConstantConfigurationProvider(FramePoint3D framePoint)
   {
      configuration = new FramePose(framePoint, new FrameQuaternion(framePoint.getReferenceFrame()));
   }

   public ConstantConfigurationProvider(FramePose framePose)
   {
      this.configuration = new FramePose(framePose);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      configuration.getPositionIncludingFrame(positionToPack);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      configuration.getOrientationIncludingFrame(orientationToPack);
   }
}
