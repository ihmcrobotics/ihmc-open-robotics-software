package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CurrentConfigurationProvider implements SE3ConfigurationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentConfigurationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.setToZero(endEffectorFrame);
   }

   @Override
   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(endEffectorFrame);
   }

   public void getPose(FramePose lastFootstepPose)
   {
      lastFootstepPose.setToZero(endEffectorFrame);
   }
}
