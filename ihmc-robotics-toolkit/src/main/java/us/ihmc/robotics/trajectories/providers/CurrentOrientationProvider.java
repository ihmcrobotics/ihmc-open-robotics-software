package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameOrientation;

public class CurrentOrientationProvider implements OrientationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentOrientationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(endEffectorFrame);
   }
}