package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ConstantConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose configuration;

   public ConstantConfigurationProvider(ReferenceFrame referenceFrame)
   {
      this.configuration = new FramePose(referenceFrame);
   }

   public ConstantConfigurationProvider(FramePoint3D framePoint)
   {
      configuration = new FramePose(framePoint, new FrameOrientation(framePoint.getReferenceFrame()));
   }

   public ConstantConfigurationProvider(FramePose framePose)
   {
      this.configuration = new FramePose(framePose);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      configuration.getPositionIncludingFrame(positionToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      configuration.getOrientationIncludingFrame(orientationToPack);
   }
}
