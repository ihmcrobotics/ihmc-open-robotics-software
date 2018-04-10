package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public interface FootPoseProvider
{
   FramePose3DReadOnly getCurrentFootPose(RobotSide robotSide);
}
