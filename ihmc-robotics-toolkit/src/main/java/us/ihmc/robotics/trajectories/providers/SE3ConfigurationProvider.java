package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface SE3ConfigurationProvider extends PositionProvider, OrientationProvider
{
   default void getPose(FramePose3DBasics poseToPack)
   {
      poseToPack.setReferenceFrame(this.getReferenceFrame());
      getPose((FixedFramePose3DBasics) poseToPack);
   }

   default void getPose(FixedFramePose3DBasics poseToPack)
   {
      poseToPack.setMatchingFrame(getPose());
   }

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
   default FrameQuaternionReadOnly getOrientation()
   {
      return getPose().getOrientation();
   }

   FramePose3DReadOnly getPose();
}
