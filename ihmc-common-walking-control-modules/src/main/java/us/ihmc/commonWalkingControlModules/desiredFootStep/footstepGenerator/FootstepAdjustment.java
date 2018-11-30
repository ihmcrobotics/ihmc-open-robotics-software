package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for adjusting generated footsteps.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FootstepAdjustment
{
   /**
    * Adjusts the footstep height, pitch, and roll.
    * 
    * @param footstepPose the generated footstep from the {@code ContinuousStepGenerator}. Height,
    *           pitch, and roll have to be computed.
    * @param footSide indicates for which foot the footstep is meant to be.
    * @return the adjusted footstep.
    */
   FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide);
}
