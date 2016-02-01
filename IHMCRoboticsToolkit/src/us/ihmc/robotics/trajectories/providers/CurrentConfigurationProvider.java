package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CurrentConfigurationProvider implements SE3ConfigurationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentConfigurationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   public void get(FramePoint positionToPack)
   {
      positionToPack.setToZero(endEffectorFrame);
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(endEffectorFrame);
   }
}
