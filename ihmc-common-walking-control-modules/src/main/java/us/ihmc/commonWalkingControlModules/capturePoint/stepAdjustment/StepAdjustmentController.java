package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;

public interface StepAdjustmentController
{
   void reset();

   void setFootstepAfterTheCurrentOne(SimpleFootstep nextFootstep, FootstepTiming nextFootstepTiming);

   void setFootstepToAdjust(SimpleFootstep footstep, double swingDuration, double nextTransferDuration);

   void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

   void setStepConstraintRegion(StepConstraintRegion stepConstraintRegion);

   boolean hasStepConstraintRegion();

   void initialize(double initialTime, RobotSide supportSide);

   void compute(double currentTime,
                       FramePoint2DReadOnly desiredICP,
                       FramePoint2DReadOnly currentICP,
                       FrameVector2DReadOnly residualICPError,
                       double omega0);

   FramePose3DReadOnly getFootstepSolution();

   boolean wasFootstepAdjusted();

   boolean useStepAdjustment();


}
