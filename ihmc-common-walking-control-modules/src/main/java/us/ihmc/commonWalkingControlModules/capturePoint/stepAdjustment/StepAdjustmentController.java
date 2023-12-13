package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.util.List;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegionsList;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;

public interface StepAdjustmentController extends SCS2YoGraphicHolder
{
   void reset();

   void setFootstepQueueInformation(int numberOfStepsInQueue, double subsequentStepDuration);

   void setFootstepToAdjust(SimpleFootstep footstep, double swingDuration);

   void submitSwingSpeedUpUnderDisturbance(double remainingTimeForSwing);

   void setStepConstraintRegions(List<StepConstraintRegion> stepConstraintRegion);

   void initialize(double initialTime, RobotSide supportSide);

   void compute(double currentTime,
                FramePoint2DReadOnly desiredICP,
                FramePoint2DReadOnly currentICP,
                double omega0,
                FramePoint2DReadOnly copToShrinkAbout,
                double percentageToShrinkPolygon);

   FramePose3DReadOnly getFootstepSolution();

   List<StepConstraintRegion> getStepConstraintRegions();

   boolean wasFootstepAdjusted();

   boolean useStepAdjustment();
}
