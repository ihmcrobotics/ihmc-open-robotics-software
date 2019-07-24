package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPTimeFreezerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class ICPPlannerWithTimeFreezerWrapper implements ICPPlannerWithTimeFreezerInterface
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix = "icpPlanner";

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoBoolean doTimeFreezing;
   private final YoBoolean isTimeBeingFrozen;
   private final YoDouble timeDelay;
   private final YoDouble capturePointPositionError;
   private final YoDouble distanceToFreezeLine;
   private final YoDouble previousTime;
   private final YoDouble freezeTimeFactor;
   private final YoDouble maxCapturePointErrorAllowedToBeginSwingPhase;
   private final YoDouble maxAllowedCapturePointErrorWithoutPartialTimeFreeze;

   private final FramePoint2D tmpCapturePointPosition;
   private final FrameVector2D tmpCapturePointVelocity;

   protected final ICPPlannerInterface icpPlanner;

   public ICPPlannerWithTimeFreezerWrapper(ICPPlannerInterface icpPlanner, ICPTimeFreezerParameters icpTimeFreezerParameters)
   {
      this.icpPlanner = icpPlanner;

      this.timeDelay = new YoDouble(namePrefix + "TimeDelayFromFreezer", registry);
      this.capturePointPositionError = new YoDouble(namePrefix + "CapturePointPositionError", registry);
      this.distanceToFreezeLine = new YoDouble(namePrefix + "DistanceToFreezeLine", registry);
      this.freezeTimeFactor = new YoDouble(namePrefix + "FreezeTimeFactor", registry);
      this.maxCapturePointErrorAllowedToBeginSwingPhase = new YoDouble(namePrefix + "MaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze = new YoDouble(namePrefix + "MaxAllowedCapturePointErrorWithoutTimeFreeze", registry);
      this.previousTime = new YoDouble(namePrefix + "PreviousTime", registry);
      this.doTimeFreezing = new YoBoolean(namePrefix + "DoTimeFreezing", registry);
      this.isTimeBeingFrozen = new YoBoolean(namePrefix + "IsTimeBeingFrozen", registry);
      this.tmpCapturePointPosition = new FramePoint2D(worldFrame);
      this.tmpCapturePointVelocity = new FrameVector2D(worldFrame);

      initializeTimeFreezerParameters(icpTimeFreezerParameters);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void initializeTimeFreezerParameters(ICPTimeFreezerParameters icpPlannerParameters)
   {
      this.maxCapturePointErrorAllowedToBeginSwingPhase.set(icpPlannerParameters.getMaxInstantaneousCapturePointErrorForStartingSwing());
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze.set(icpPlannerParameters.getMaxAllowedErrorWithoutPartialTimeFreeze());
      this.freezeTimeFactor.set(icpPlannerParameters.getFreezeTimeFactor());

      this.doTimeFreezing.set(icpPlannerParameters.getDoTimeFreezing());

      this.isTimeBeingFrozen.set(false);
      this.timeDelay.set(0.0);
      this.capturePointPositionError.set(0.0);
      this.distanceToFreezeLine.set(0.0);
   }

   /** {@inheritDoc} */
   @Override
   public void clearPlan()
   {
      icpPlanner.clearPlan();
   }

   /** {@inheritDoc} */
   @Override
   public void setSupportLeg(RobotSide robotside)
   {
      icpPlanner.setSupportLeg(robotside);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferToSide(RobotSide robotSide)
   {
      icpPlanner.setTransferToSide(robotSide);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferFromSide(RobotSide robotSide)
   {
      icpPlanner.setTransferFromSide(robotSide);
   }

   /** {@inheritDoc} */
   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions)
   {
      icpPlanner.addFootstepToPlan(footstep, timing, shiftFractions);
   }

   /** {@inheritDoc} */
   @Override
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   {
      icpPlanner.holdCurrentICP(icpPositionToHold);
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForStanding(double initialTime)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      icpPlanner.initializeForStanding(initialTime);
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForTransfer(double initialTime)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      icpPlanner.initializeForTransfer(initialTime);
   }

   /** {@inheritDoc} */
   @Override
   public void computeFinalCoMPositionInTransfer()
   {
      icpPlanner.computeFinalCoMPositionInTransfer();
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForSingleSupport(double initialTime)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      icpPlanner.initializeForSingleSupport(initialTime);
   }

   /** {@inheritDoc} */
   @Override
   public void computeFinalCoMPositionInSwing()
   {
      icpPlanner.computeFinalCoMPositionInSwing();
   }

   /** {@inheritDoc} */
   @Override
   public void updateCurrentPlan()
   {
      timeDelay.set(0.0);
      previousTime.set(getTimeInCurrentState());
      icpPlanner.updateCurrentPlan();
   }

   /** {@inheritDoc} */
   @Override
   public double estimateTimeRemainingForStateUnderDisturbance(FramePoint2DReadOnly actualCapturePointPosition)
   {
      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(actualCapturePointPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void compute(double time)
   {
      throw new RuntimeException("Use the method ICPPlannerWithTimeFreezer.compute(FramePoint2D, double) instead. If the time freeze feature is not desired, use ContinuousCMPBasedICPPlanner instead.");
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointPosition(FramePoint3D desiredCapturePointPositionToPack)
   {
      icpPlanner.getDesiredCapturePointPosition(desiredCapturePointPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointPosition(FramePoint2D desiredCapturePointPositionToPack)
   {
      icpPlanner.getDesiredCapturePointPosition(desiredCapturePointPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointPosition(YoFramePoint3D desiredCapturePointPositionToPack)
   {
      icpPlanner.getDesiredCapturePointPosition(desiredCapturePointPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfMassPosition(FramePoint3D desiredCenterOfMassPositionToPack)
   {
      icpPlanner.getDesiredCenterOfMassPosition(desiredCenterOfMassPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfMassPosition(YoFramePoint3D desiredCenterOfMassPositionToPack)
   {
      icpPlanner.getDesiredCenterOfMassPosition(desiredCenterOfMassPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector3D desiredCapturePointVelocityToPack)
   {
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack)
   {
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(YoFrameVector3D desiredCapturePointVelocityToPack)
   {
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointAcceleration(FrameVector3D desiredCapturePointAccelerationToPack)
   {
      icpPlanner.getDesiredCapturePointAcceleration(desiredCapturePointAccelerationToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint3D desiredCentroidalMomentumPivotPositionToPack)
   {
      icpPlanner.getDesiredCentroidalMomentumPivotPosition(desiredCentroidalMomentumPivotPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack)
   {
      icpPlanner.getDesiredCentroidalMomentumPivotPosition(desiredCentroidalMomentumPivotPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector3D desiredCentroidalMomentumPivotVelocityToPack)
   {
      icpPlanner.getDesiredCentroidalMomentumPivotVelocity(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2D desiredCentroidalMomentumPivotVelocityToPack)
   {
      icpPlanner.getDesiredCentroidalMomentumPivotVelocity(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressurePosition(FramePoint3D desiredCenterOfPressurePositionToPack)
   {
      icpPlanner.getDesiredCenterOfPressurePosition(desiredCenterOfPressurePositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressurePosition(FramePoint2D desiredCenterOfPressurePositionToPack)
   {
      icpPlanner.getDesiredCenterOfPressurePosition(desiredCenterOfPressurePositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressureVelocity(FrameVector3D desiredCenterOfPressureVelocityToPack)
   {
      icpPlanner.getDesiredCenterOfPressureVelocity(desiredCenterOfPressureVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressureVelocity(FrameVector2D desiredCenterOfPressureVelocityToPack)
   {
      icpPlanner.getDesiredCenterOfPressureVelocity(desiredCenterOfPressureVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentState()
   {
      return icpPlanner.getTimeInCurrentState();
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentStateRemaining()
   {
      return icpPlanner.getTimeInCurrentStateRemaining();
   }

   /** {@inheritDoc} */
   @Override
   public double getCurrentStateDuration()
   {
      return icpPlanner.getCurrentStateDuration();
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferDuration(int stepNumber, double duration)
   {
      icpPlanner.setTransferDuration(stepNumber, duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setSwingDuration(int stepNumber, double duration)
   {
      icpPlanner.setSwingDuration(stepNumber, duration);
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferDuration(int stepNumber)
   {
      return icpPlanner.getTransferDuration(stepNumber);
   }

   /** {@inheritDoc} */
   @Override
   public double getSwingDuration(int stepNumber)
   {
      return icpPlanner.getSwingDuration(stepNumber);
   }

   /** {@inheritDoc} */
   @Override
   public void setFinalTransferDuration(double duration)
   {
      icpPlanner.setFinalTransferDuration(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setFinalTransferDurationAlpha(double durationAlpha)
   {
      icpPlanner.setFinalTransferDurationAlpha(durationAlpha);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferDurationAlpha(int stepNubmer, double transferDurationAlpha)
   {
      icpPlanner.setTransferDurationAlpha(stepNubmer, transferDurationAlpha);
   }

   /** {@inheritDoc} */
   @Override
   public void setSwingDurationAlpha(int stepNubmer, double transferDurationAlpha)
   {
      icpPlanner.setSwingDurationAlpha(stepNubmer, transferDurationAlpha);
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferDurationAlpha(int stepNumber)
   {
      return icpPlanner.getTransferDurationAlpha(stepNumber);
   }

   /** {@inheritDoc} */
   @Override
   public double getSwingDurationAlpha(int stepNumber)
   {
      return icpPlanner.getSwingDurationAlpha(stepNumber);
   }

   /** {@inheritDoc} */
   @Override
   public double getInitialTime()
   {
      return icpPlanner.getInitialTime();
   }

   /** {@inheritDoc} */
   @Override
   public void setOmega0(double omega0)
   {
      icpPlanner.setOmega0(omega0);
   }

   /** {@inheritDoc} */
   @Override
   public boolean isInDoubleSupport()
   {
      return icpPlanner.isInDoubleSupport();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isInStanding()
   {
      return icpPlanner.isInStanding();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isInInitialTransfer()
   {
      return icpPlanner.isInInitialTransfer();
   }

   /** {@inheritDoc} */
   @Override
   public void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getFinalDesiredCapturePointPosition(YoFramePoint2D finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getFinalDesiredCenterOfMassPosition(FramePoint3D finalDesiredCenterOfMassPositionToPack)
   {
      icpPlanner.getFinalDesiredCenterOfMassPosition(finalDesiredCenterOfMassPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getNextExitCMP(FramePoint3D exitCMPToPack)
   {
      icpPlanner.getNextExitCMP(exitCMPToPack);
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDone()
   {
      return icpPlanner.isDone();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isOnExitCMP()
   {
      return icpPlanner.isOnExitCMP();
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return icpPlanner.getNumberOfFootstepsToConsider();
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsRegistered()
   {
      return icpPlanner.getNumberOfFootstepsRegistered();
   }

   /** {@inheritDoc} */
   @Override
   public RobotSide getTransferToSide()
   {
      return icpPlanner.getTransferToSide();
   }

   /** {@inheritDoc} */
   @Override
   public double getOmega0()
   {
      return icpPlanner.getOmega0();
   }

   /** {@inheritDoc} */
   @Override
   public boolean getIsTimeBeingFrozen()
   {
      return isTimeBeingFrozen.getBooleanValue();
   }


   /** {@inheritDoc} */
   @Override
   public void compute(FramePoint2DReadOnly currentCapturePointPosition, double time)
   {
      icpPlanner.compute(time - timeDelay.getDoubleValue());

      if (doTimeFreezing.getBooleanValue())
      {
         icpPlanner.getDesiredCapturePointPosition(tmpCapturePointPosition);
         icpPlanner.getDesiredCapturePointVelocity(tmpCapturePointVelocity);
         doTimeFreezeIfNeeded(currentCapturePointPosition, time);
      }

      previousTime.set(time);
   }



   private void doTimeFreezeIfNeeded(FramePoint2DReadOnly currentCapturePointPosition, double time)
   {
      computeCapturePointDistantToFreezeLine(currentCapturePointPosition, tmpCapturePointPosition, tmpCapturePointVelocity);

      if (isDone())
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if (getTimeInCurrentStateRemaining() < 0.1 && isInDoubleSupport()
            && distanceToFreezeLine.getDoubleValue() > maxCapturePointErrorAllowedToBeginSwingPhase.getDoubleValue())
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if ((distanceToFreezeLine.getDoubleValue() > maxAllowedCapturePointErrorWithoutPartialTimeFreeze.getDoubleValue()))
      {
         freezeTimeUsingFreezeTimeFactor(time);
         isTimeBeingFrozen.set(true);
      }
      else
      {
         isTimeBeingFrozen.set(false);
      }
   }

   private void freezeTimeUsingFreezeTimeFactor(double time)
   {
      timeDelay.add(freezeTimeFactor.getDoubleValue() * (time - previousTime.getDoubleValue()));
   }

   private void completelyFreezeTime(double time)
   {
      timeDelay.add(time - previousTime.getDoubleValue());
   }

   private void computeCapturePointDistantToFreezeLine(FramePoint2DReadOnly currentCapturePointPosition, FramePoint2DReadOnly desiredCapturePointPosition,
                                                       FrameVector2DReadOnly desiredCapturePointVelocity)
   {
      distanceToFreezeLine.set(CapturePointTools.computeDistanceToCapturePointFreezeLineIn2d(currentCapturePointPosition, desiredCapturePointPosition,
                                                                                             desiredCapturePointVelocity));
   }
}
