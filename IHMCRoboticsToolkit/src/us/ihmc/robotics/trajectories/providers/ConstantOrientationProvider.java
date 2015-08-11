package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;

/**
 * @author twan
 *         Date: 5/29/13
 */
public class ConstantOrientationProvider implements OrientationProvider
{
   private final FrameOrientation orientation;

   public ConstantOrientationProvider(FrameOrientation orientation)
   {
      this.orientation = new FrameOrientation(orientation);
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }
}
