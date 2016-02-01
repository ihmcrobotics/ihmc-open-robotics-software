package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CurrentOrientationProvider implements OrientationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentOrientationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(endEffectorFrame);
   }
}