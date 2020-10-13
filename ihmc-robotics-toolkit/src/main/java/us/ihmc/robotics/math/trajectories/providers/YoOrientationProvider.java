package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;


public class YoOrientationProvider implements OrientationProvider
{
   private final YoFrameYawPitchRoll frameOrientation;

   public YoOrientationProvider(YoFrameYawPitchRoll frameOrientation)
   {
      this.frameOrientation = frameOrientation;
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(frameOrientation);
   }
}
