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
    * Checks if the given pose is steppable.
    *
    * @param solePose footstep to check
    * @param robotSide side of the given footstep
    */
   boolean isFootstepValid(FramePose3DReadOnly solePose, RobotSide robotSide);
}
