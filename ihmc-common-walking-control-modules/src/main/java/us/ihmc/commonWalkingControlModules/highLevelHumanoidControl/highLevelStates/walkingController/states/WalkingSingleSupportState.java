package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingSingleSupportState extends SingleSupportState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Footstep nextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private double swingTime;

   private static final int additionalFootstepsToConsider = 2;
   private final Footstep[] footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);

   private final FramePose actualFootPoseInWorld = new FramePose(worldFrame);
   private final FramePose desiredFootPoseInWorld = new FramePose(worldFrame);
   private final FramePoint3D nextExitCMP = new FramePoint3D();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final LegConfigurationManager legConfigurationManager;

   private final YoDouble fractionOfSwingToStraightenSwingLeg = new YoDouble("fractionOfSwingToStraightenSwingLeg", registry);
   private final YoDouble fractionOfSwingToCollapseStanceLeg = new YoDouble("fractionOfSwingToCollapseStanceLeg", registry);

   private final YoDouble remainingSwingTimeAccordingToPlan = new YoDouble("remainingSwingTimeAccordingToPlan", registry);
   private final YoDouble estimatedRemainingSwingTimeUnderDisturbance = new YoDouble("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final YoDouble optimizedRemainingSwingTime = new YoDouble("optimizedRemainingSwingTime", registry);
   private final YoDouble icpErrorThresholdToSpeedUpSwing = new YoDouble("icpErrorThresholdToSpeedUpSwing", registry);

   private final YoBoolean finishSingleSupportWhenICPPlannerIsDone = new YoBoolean("finishSingleSupportWhenICPPlannerIsDone", registry);
   private final YoBoolean minimizeAngularMomentumRateZDuringSwing = new YoBoolean("minimizeAngularMomentumRateZDuringSwing", registry);

   private final FrameVector3D touchdownErrorVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public WalkingSingleSupportState(RobotSide supportSide, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
         HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
         WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(supportSide, WalkingStateEnum.getWalkingSingleSupportState(supportSide), walkingMessageHandler, controllerToolbox, managerFactory,
            parentRegistry);

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      fractionOfSwingToStraightenSwingLeg.set(walkingControllerParameters.getLegConfigurationParameters().getFractionOfSwingToStraightenLeg());
      fractionOfSwingToCollapseStanceLeg.set(walkingControllerParameters.getLegConfigurationParameters().getFractionOfSwingToCollapseStanceLeg());

      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());
      finishSingleSupportWhenICPPlannerIsDone.set(walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone());
      minimizeAngularMomentumRateZDuringSwing.set(walkingControllerParameters.minimizeAngularMomentumRateZDuringSwing());

      setYoVariablesToNaN();
   }

   private void setYoVariablesToNaN()
   {
      optimizedRemainingSwingTime.setToNaN();
      estimatedRemainingSwingTimeUnderDisturbance.setToNaN();
      remainingSwingTimeAccordingToPlan.setToNaN();
   }


   @Override
   public void doAction()
   {
      super.doAction();

      boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();
      boolean requestSwingSpeedUp = icpErrorIsTooLarge;

      if (walkingMessageHandler.hasRequestedFootstepAdjustment())
      {
         boolean footstepHasBeenAdjusted = walkingMessageHandler.pollRequestedFootstepAdjustment(nextFootstep);
         if (footstepHasBeenAdjusted)
         {
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.replanSwingTrajectory(swingSide, nextFootstep, swingTime, true);

            balanceManager.updateCurrentICPPlan();
         }

      }
      else if (balanceManager.useICPOptimization()) // TODO figure out a way of combining the two following modules
      {
         boolean footstepIsBeingAdjusted = balanceManager.checkAndUpdateFootstepFromICPOptimization(nextFootstep);

         if (footstepIsBeingAdjusted)
         {
            requestSwingSpeedUp = true;
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.replanSwingTrajectory(swingSide, nextFootstep, swingTime, true);

            balanceManager.updateCurrentICPPlan();
            //legConfigurationManager.prepareForLegBracing(swingSide);
         }
      }
      else if (balanceManager.isPushRecoveryEnabled())
      {
         boolean footstepHasBeenAdjusted = balanceManager.checkAndUpdateFootstep(nextFootstep);
         if (footstepHasBeenAdjusted)
         {
            requestSwingSpeedUp = true;
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.replanSwingTrajectory(swingSide, nextFootstep, swingTime, false);

            walkingMessageHandler.reportWalkingAbortRequested();
            walkingMessageHandler.clearFootsteps();

            double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
            double swingDuration = footstepTiming.getSwingTime();
            double transferDuration = footstepTiming.getTransferTime();

            balanceManager.clearICPPlan();
            balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);
            balanceManager.setICPPlanSupportSide(supportSide);
            balanceManager.initializeICPPlanForSingleSupport(swingDuration, transferDuration, finalTransferTime);
         }
      }

      if (requestSwingSpeedUp)
      {
         double swingTimeRemaining = requestSwingSpeedUpIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }
      boolean feetAreWellPositioned = legConfigurationManager.areFeetWellPositionedForCollapse(swingSide.getOppositeSide(), nextFootstep.getSoleReferenceFrame());

      if (getTimeInCurrentState() > fractionOfSwingToStraightenSwingLeg.getDoubleValue() * swingTime)
      {
         legConfigurationManager.straightenLegDuringSwing(swingSide);
      }
      if (getTimeInCurrentState() > fractionOfSwingToCollapseStanceLeg.getDoubleValue() * swingTime && !legConfigurationManager.isLegCollapsed(supportSide) &&
            feetAreWellPositioned)
      {
         legConfigurationManager.collapseLegDuringSwing(swingSide.getOppositeSide());
      }

      walkingMessageHandler.clearFootTrajectory();

      switchToToeOffIfPossible(supportSide);
   }

   @Override
   public boolean isDone()
   {
      if (super.isDone())
      {
         return true;
      }

      return finishSingleSupportWhenICPPlannerIsDone.getBooleanValue() && balanceManager.isICPPlanDone();
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      double defaultSwingTime = walkingMessageHandler.getDefaultSwingTime();
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double defaultTouchdownTime = walkingMessageHandler.getDefaultTouchdownTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         swingTime = defaultSwingTime;
         footstepTiming.setTimings(swingTime, defaultTouchdownTime, defaultTransferTime);
         balanceManager.packFootstepForRecoveringFromDisturbance(swingSide, defaultSwingTime, nextFootstep);
         nextFootstep.setTrajectoryType(TrajectoryType.DEFAULT);
         walkingMessageHandler.reportWalkingAbortRequested();
         walkingMessageHandler.clearFootsteps();
      }
      else
      {
         swingTime = walkingMessageHandler.getNextSwingTime();
         walkingMessageHandler.poll(nextFootstep, footstepTiming);
      }

      updateFootstepParameters();

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringSwing.getBooleanValue());
      balanceManager.setNextFootstep(nextFootstep);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      boolean isLastStep = stepsToAdd == 0;
      for (int i = 0; i < stepsToAdd; i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps[i]);
         walkingMessageHandler.peekTiming(i, footstepTimings[i]);
         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i]);
      }

      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport(footstepTiming.getSwingTime(), footstepTiming.getTransferTime(), finalTransferTime);

      if (balanceManager.wasTimingAdjustedForReachability())
      {
         double currentTransferDuration = balanceManager.getCurrentTransferDurationAdjustedForReachability();
         double currentSwingDuration = balanceManager.getCurrentSwingDurationAdjustedForReachability();
         double nextTransferDuration = balanceManager.getNextTransferDurationAdjustedForReachability();
         double currentTouchdownDuration = balanceManager.getCurrentTouchdownDuration();

         swingTime = currentSwingDuration;
         footstepTiming.setTimings(currentSwingDuration, currentTouchdownDuration, currentTransferDuration);

         if (isLastStep)
         {
            balanceManager.setFinalTransferTime(nextTransferDuration);
         }
         else
         {
            double nextSwingTime = footstepTimings[0].getSwingTime();
            footstepTimings[0].setTimings(nextSwingTime, currentTouchdownDuration, nextTransferDuration);
            walkingMessageHandler.adjustTimings(0, nextSwingTime, currentTouchdownDuration, nextTransferDuration);
         }
      }

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         balanceManager.updateCurrentICPPlan();
         balanceManager.requestICPPlannerToHoldCurrentCoMInNextDoubleSupport();
      }

      double touchdownTime = footstepTiming.getTouchdownDuration();
      feetManager.requestSwing(swingSide, nextFootstep, swingTime, touchdownTime);

      legConfigurationManager.startSwing(swingSide);
      legConfigurationManager.useHighWeight(swingSide.getOppositeSide());
      legConfigurationManager.setStepDuration(supportSide, footstepTiming.getStepTime());

      if (isLastStep)
      {
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, finalTransferTime, 0.0);
      }
      else
      {
         FootstepTiming nextTiming = footstepTimings[0];
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, nextTiming.getTransferTime(), nextTiming.getSwingTime());
      }

      nextFootstep.getPose(desiredFootPoseInWorld);
      desiredFootPoseInWorld.changeFrame(worldFrame);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepStarted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      balanceManager.minimizeAngularMomentumRateZ(false);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);

      actualFootPoseInWorld.checkReferenceFrameMatch(desiredFootPoseInWorld);
      touchdownErrorVector.sub(actualFootPoseInWorld.getPosition(), desiredFootPoseInWorld.getPosition());
      touchdownErrorVector.setZ(0.0);
      walkingMessageHandler.addOffsetVector(touchdownErrorVector);

      walkingMessageHandler.reportFootstepCompleted(swingSide, actualFootPoseInWorld);
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);

      setYoVariablesToNaN();
   }

   private final FramePoint2D filteredDesiredCoP = new FramePoint2D(worldFrame);
   private final FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
   private final FramePoint2D desiredCoP = new FramePoint2D(worldFrame);
   private final FramePoint2D desiredICP = new FramePoint2D(worldFrame);
   private final FramePoint2D currentICP = new FramePoint2D(worldFrame);
   public void switchToToeOffIfPossible(RobotSide supportSide)
   {
      boolean shouldComputeToeLineContact = feetManager.shouldComputeToeLineContact();
      boolean shouldComputeToePointContact = feetManager.shouldComputeToePointContact();

      if (shouldComputeToeLineContact || shouldComputeToePointContact)
      {
         balanceManager.getDesiredCMP(desiredCMP);
         balanceManager.getDesiredICP(desiredICP);
         balanceManager.getCapturePoint(currentICP);
         balanceManager.getNextExitCMP(nextExitCMP);

         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(supportSide), desiredCoP);
         controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(supportSide), filteredDesiredCoP);

         feetManager.updateToeOffStatusSingleSupport(nextFootstep, nextExitCMP, desiredCMP, desiredCoP, desiredICP, currentICP);

         if (feetManager.okForPointToeOff() && shouldComputeToePointContact)
            feetManager.requestPointToeOff(supportSide, nextExitCMP, filteredDesiredCoP);
         else if (feetManager.okForLineToeOff() && shouldComputeToeLineContact)
            feetManager.requestLineToeOff(supportSide, nextExitCMP, filteredDesiredCoP);
      }
   }

   /**
    * Request the swing trajectory to speed up using {@link us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface#estimateTimeRemainingForStateUnderDisturbance(FramePoint2d)}.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @return the current swing time remaining for the swing foot trajectory
    */
   private double requestSwingSpeedUpIfNeeded()
   {
      remainingSwingTimeAccordingToPlan.set(balanceManager.getTimeRemainingInCurrentState());

      double remainingTime = balanceManager.estimateTimeRemainingForSwingUnderDisturbance();
      estimatedRemainingSwingTimeUnderDisturbance.set(remainingTime);

      if (remainingTime > 1.0e-3)
      {
         double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / remainingTime;
         return feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
      }
      else if (remainingSwingTimeAccordingToPlan.getDoubleValue() > 1.0e-3)
      {
         return feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
      }
      return remainingSwingTimeAccordingToPlan.getDoubleValue();
   }

   private void updateFootstepParameters()
   {
      feetManager.adjustHeightIfNeeded(nextFootstep);

      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setUpcomingFootstep(nextFootstep);
      pelvisOrientationManager.updateTrajectoryFromFootstep(); // fixme this shouldn't be called when the footstep is updated

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
      transferToAndNextFootstepsData.setTransferFromDesiredFootstep(walkingMessageHandler.getLastDesiredFootstep(supportSide));
      double extraToeOffHeight = 0.0;
      if (feetManager.canDoSingleSupportToeOff(nextFootstep, swingSide))
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);

      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodies.
      controllerToolbox.updateContactPointsForUpcomingFootstep(nextFootstep);
      controllerToolbox.updateBipedSupportPolygons();
   }

   @Override
   protected boolean hasMinimumTimePassed()
   {
      double minimumSwingTime;
      if (balanceManager.isRecoveringFromDoubleSupportFall())
         minimumSwingTime = 0.15;
      else
         minimumSwingTime = swingTime * minimumSwingFraction.getDoubleValue();

      return getTimeInCurrentState() > minimumSwingTime;
   }
}