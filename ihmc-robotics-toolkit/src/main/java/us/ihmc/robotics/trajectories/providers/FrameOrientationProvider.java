package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface FrameOrientationProvider extends ReferenceFrameHolder
{
   default ReferenceFrame getReferenceFrame()
   {
      return getOrientation().getReferenceFrame();
   }

   FrameQuaternionReadOnly getOrientation();
}