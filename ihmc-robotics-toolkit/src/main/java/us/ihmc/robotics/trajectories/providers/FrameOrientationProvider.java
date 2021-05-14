package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface FrameOrientationProvider extends ReferenceFrameHolder, OrientationProvider
{
   default ReferenceFrame getReferenceFrame()
   {
      return getOrientation().getReferenceFrame();
   }

   @Override
   FrameOrientation3DReadOnly getOrientation();
}