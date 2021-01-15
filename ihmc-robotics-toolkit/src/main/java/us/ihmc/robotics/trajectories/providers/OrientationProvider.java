package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface OrientationProvider extends ReferenceFrameHolder
{
   default void getOrientation(FrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setReferenceFrame(this.getReferenceFrame());
      getOrientation((FixedFrameQuaternionBasics) orientationToPack);
   }

   default void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setMatchingFrame(getOrientation());
   }

   default ReferenceFrame getReferenceFrame()
   {
      return getOrientation().getReferenceFrame();
   }

   FrameQuaternionReadOnly getOrientation();
}