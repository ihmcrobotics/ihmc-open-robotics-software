package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ConstantConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose configuration;

   public ConstantConfigurationProvider(ReferenceFrame referenceFrame)
   {
      this.configuration = new FramePose(referenceFrame);
   }

   public ConstantConfigurationProvider(FramePoint framePoint)
   {
      configuration = new FramePose(framePoint, new FrameOrientation(framePoint.getReferenceFrame()));
   }

   public ConstantConfigurationProvider(FramePose framePose)
   {
      this.configuration = new FramePose(framePose);
   }

   public void get(FramePoint positionToPack)
   {
      configuration.getPositionIncludingFrame(positionToPack);
   }

   public void get(FrameOrientation orientationToPack)
   {
      configuration.getOrientationIncludingFrame(orientationToPack);
   }
}
