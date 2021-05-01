package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface FramePoseProvider extends FramePositionProvider, FrameOrientationProvider, PoseProvider
{
   @Override
   default ReferenceFrame getReferenceFrame()
   {
      return getPose().getReferenceFrame();
   }

   @Override
   default FramePoint3DReadOnly getPosition()
   {
      return getPose().getPosition();
   }

   @Override
   default FrameOrientation3DReadOnly getOrientation()
   {
      return getPose().getOrientation();
   }

   FramePose3DReadOnly getPose();
}
