package us.ihmc.robotics.trajectories.providers;

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

   FramePose3DReadOnly getPose();

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
}
