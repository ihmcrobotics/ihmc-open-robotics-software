package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;

/**
 * @author twan
 *         Date: 5/29/13
 *         Note this class is only used in tests
 */
@Deprecated
public class ConstantOrientationProvider implements OrientationProvider
{
   private final FrameQuaternion orientation;

   public ConstantOrientationProvider(FrameQuaternion orientation)
   {
      this.orientation = new FrameQuaternion(orientation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return getOrientation().getReferenceFrame();
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return orientation;
   }
}
