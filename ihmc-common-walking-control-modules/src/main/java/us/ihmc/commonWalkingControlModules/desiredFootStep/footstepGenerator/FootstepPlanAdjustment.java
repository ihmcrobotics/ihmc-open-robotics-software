package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for adjusting generated footsteps for an entire plan, after the steps have been generated.
 * </p>
 * 
 * @author Robert Griffin
 */
public interface FootstepPlanAdjustment
{
   /**
    * Adjusts the footstep height, pitch, and roll for the entire plan.
    *
    * @param stanceFootPose                   the pose of the stance foot while this plan is being generated. Not Modified.
    * @param startingIndexToAdjust            index of the first step in the plan that should be adjusted. Not modified
    * @param footstepDataListMessageToAdjust  all the footsteps that are adjusted by this class. Modified.
    */
   void adjustFootstepPlan(FramePose3DReadOnly stanceFootPose, int startingIndexToAdjust, FootstepDataListMessage footstepDataListMessageToAdjust);
}
