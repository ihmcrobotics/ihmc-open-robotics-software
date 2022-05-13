package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.StepConstraintsListMessage;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide the given constraint regions for a foothold.
 * </p>
 */
public interface StepConstraintRegionCalculator
{
   /**
    * Calculates the step constraint regions for a footstep
    *
    * @param stanceFootPose stance foot pose
    * @param footstepPose step pose
    * @param stepConstraintsToPack step constraint regions to return. Modified.
    */
   void computeConstraintRegions(FramePose3DReadOnly stanceFootPose, FramePose3DReadOnly footstepPose, StepConstraintsListMessage stepConstraintsToPack);
}
