package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;


public class YoOrientationProvider implements OrientationProvider
{
   private final YoFrameOrientation frameOrientation;

   public YoOrientationProvider(YoFrameOrientation frameOrientation)
   {
      this.frameOrientation = frameOrientation;
   }

   public void get(FrameOrientation orientationToPack)
   {
      frameOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }
}
