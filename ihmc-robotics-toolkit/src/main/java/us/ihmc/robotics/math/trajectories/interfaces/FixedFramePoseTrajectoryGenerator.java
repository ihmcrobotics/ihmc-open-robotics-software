package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.commons.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public interface FixedFramePoseTrajectoryGenerator
      extends FixedFramePositionTrajectoryGenerator, FixedFrameOrientationTrajectoryGenerator, PoseTrajectoryGenerator
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
