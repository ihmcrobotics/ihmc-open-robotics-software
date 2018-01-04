package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CurrentOrientationProvider implements OrientationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentOrientationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(endEffectorFrame);
   }
}