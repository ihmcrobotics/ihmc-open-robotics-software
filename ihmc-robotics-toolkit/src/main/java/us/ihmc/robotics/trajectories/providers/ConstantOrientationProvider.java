package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;

/**
 * @author twan
 *         Date: 5/29/13
 */
public class ConstantOrientationProvider implements OrientationProvider
{
   private final FrameQuaternion orientation;

   public ConstantOrientationProvider(FrameQuaternion orientation)
   {
      this.orientation = new FrameQuaternion(orientation);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }
}
