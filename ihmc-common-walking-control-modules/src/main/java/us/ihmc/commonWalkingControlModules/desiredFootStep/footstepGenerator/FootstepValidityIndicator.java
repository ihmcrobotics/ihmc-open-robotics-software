package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to validate that candidate steps are steppable
 * </p>
 *
 * @author Stephen McCrory
 */
public interface FootstepValidityIndicator
{
   /**
    * Checks if the given touchdown pose is steppable given the stance pose.
    *
    * @param touchdownPose touchdown pose of the step
    * @param stanceFootPose stance pose of the step
    * @param robotSide side of the swing foot
    */
   boolean isFootstepValid(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stanceFootPose, RobotSide robotSide);
}
