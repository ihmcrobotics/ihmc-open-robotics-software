package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPPlanner
{
   public void clearPlan();

   public void setSupportLeg(RobotSide robotSide);

   public void setTransferToSide(RobotSide robotSide);

   public void setTransferFromSide(RobotSide robotSide);

   public void setOmega0(double omega0);

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   public void setDesiredCapturePointState(FramePoint2d currentDesiredCapturePointPosition, FrameVector2d currentDesiredCapturePointVelocity);

   public void setDesiredCapturePointState(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity);

   public void setDesiredCapturePointState(YoFramePoint currentDesiredCapturePointPosition, YoFrameVector currentDesiredCapturePointVelocity);

   public void setDesiredCapturePointState(YoFramePoint2d currentDesiredCapturePointPosition, YoFrameVector2d currentDesiredCapturePointVelocity);

   public void holdCurrentICP(double initialTime, FramePoint actualICPToHold);

   public void initializeForStanding(double initialTime);

   public void initializeForTransfer(double initialTime);

   public void initializeForSingleSupport(double initialTime);

   public void updateCurrentPlan();

   public void getDesiredCapturePointPositionAndVelocity(FramePoint2d desiredCapturePointPositionToPack, FrameVector2d desiredCapturePointVelocityToPack, double time);

   public void getDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack, double time);

   public void getDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack, double time);

   public void getDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack);

   public void getSingleSupportInitialCapturePointPosition(FramePoint capturePointPositionToPack);

   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack);

   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack);

   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack);

   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector desiredCentroidalMomentumPivotVelocityToPack);

   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2d desiredCentroidalMomentumPivotVelocityToPack);

   public void getNextExitCMP(FramePoint entryCMPToPack);

   public double computeAndReturnTimeRemaining(double time);

   public double computeAndReturnTimeInCurrentState(double time);

   public void setFinalTransferTime(double time);

   public boolean isInDoubleSupport();

   public boolean isInStanding();

   public boolean isInInitialTranfer();

   public boolean isDone(double time);

   public boolean isOnExitCMP();

   public void updatePlanForSingleSupportDisturbances(double time, FramePoint2d actualCapturePointPosition);

   public double estimateTimeRemainingForStateUnderDisturbance(double time, FramePoint2d actualCapturePointPosition);

   public void setMinimumSingleSupportTimeForDisturbanceRecovery(double minTime);
}
