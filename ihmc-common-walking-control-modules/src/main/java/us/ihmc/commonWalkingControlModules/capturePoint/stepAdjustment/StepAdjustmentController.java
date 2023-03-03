package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.util.List;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;

public interface StepAdjustmentController
{
   void reset();

   void setFootstepAfterTheCurrentOne(SimpleFootstep nextFootstep, FootstepTiming nextFootstepTiming);

   void setFootstepToAdjust(SimpleFootstep footstep, double swingDuration, double nextTransferDuration);

   void submitSwingSpeedUpUnderDisturbance(double remainingTimeForSwing);

   void setStepConstraintRegions(List<StepConstraintRegion> stepConstraintRegion);

   void initialize(double initialTime, RobotSide supportSide);

   void compute(double currentTime,
                       FramePoint2DReadOnly desiredICP,
                       FramePoint2DReadOnly currentICP,
                       double omega0);

   FramePose3DReadOnly getFootstepSolution();

   boolean wasFootstepAdjusted();

   boolean useStepAdjustment();

   YoGraphicDefinition getSCS2YoGraphics();
}
