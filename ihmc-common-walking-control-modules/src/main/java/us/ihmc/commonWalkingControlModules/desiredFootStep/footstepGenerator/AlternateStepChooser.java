package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used as a fallback if {@link FootstepValidityIndicator} indicates the default step location is not steppable.
 * </p>
 *
 * @author Stephen McCrory
 */
public interface AlternateStepChooser
{
   /**
    * Calculates a new step pose to be used as a replacement for the default step pose
    *
    * @param stanceFootPose stance foot pose
    * @param defaultTouchdownPose default step pose
    * @param swingSide swing side of the step to be calculated
    * @param touchdownPoseToPack the pose used to store the new calculated step. Modified.
    */
   void computeStep(FramePose2DReadOnly stanceFootPose, FramePose2DReadOnly defaultTouchdownPose, RobotSide swingSide, FramePose3D touchdownPoseToPack);
}
