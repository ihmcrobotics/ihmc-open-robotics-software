package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for obtaining the current pose of the robot feet.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FootPoseProvider
{
   /**
    * Gets the current pose of a robot sole frame.
    * 
    * @param robotSide the side of interest.
    * @return the current pose of the foot sole.
    */
   FramePose3DReadOnly getCurrentFootPose(RobotSide robotSide);
}
