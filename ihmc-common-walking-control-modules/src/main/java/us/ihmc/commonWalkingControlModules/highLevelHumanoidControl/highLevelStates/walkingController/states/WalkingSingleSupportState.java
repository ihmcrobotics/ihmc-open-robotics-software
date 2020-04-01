package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingSingleSupportState extends SingleSupportState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Footstep nextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private final FootstepShiftFractions footstepShiftFraction = new FootstepShiftFractions();
   private double swingTime;

   private static final int additionalFootstepsToConsider = 2;
   private final Footstep[] footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);
   private final FootstepShiftFractions[] footstepShiftFractions = FootstepShiftFractions.createShiftFractions(additionalFootstepsToConsider);

   private final FramePose3D actualFootPoseInWorld = new FramePose3D(worldFrame);
   private final FramePose3D desiredFootPoseInWorld = new FramePose3D(worldFrame);
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
   private final BooleanProvider minimizeAngularMomentumRateZDuringSwing;

   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   private final TouchdownErrorCompensator touchdownErrorCompensator;

   public WalkingSingleSupportState(WalkingStateEnum stateEnum, WalkingMessageHandler walkingMessageHandler, TouchdownErrorCompensator touchdownErrorCompensator,
                                    HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                                    WalkingControllerParameters walkingControllerParameters, WalkingFailureDetectionControlModule failureDetectionControlModule,
                                    YoVariableRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, parentRegistry);

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      fractionOfSwingToStraightenSwingLeg.set(walkingControllerParameters.getLegConfigurationParameters().getFractionOfSwingToStraightenLeg());
      fractionOfSwingToCollapseStanceLeg.set(walkingControllerParameters.getLegConfigurationParameters().getFractionOfSwingToCollapseStanceLeg());

      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());
      finishSingleSupportWhenICPPlannerIsDone.set(walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone());
      minimizeAngularMomentumRateZDuringSwing = new BooleanParameter("minimizeAngularMomentumRateZDuringSwing", registry,
                                                                     walkingControllerParameters.minimizeAngularMomentumRateZDuringSwing());

      setYoVariablesToNaN();
   }

   private void setYoVariablesToNaN()
   {
      optimizedRemainingSwingTime.setToNaN();
      estimatedRemainingSwingTimeUnderDisturbance.setToNaN();
      remainingSwingTimeAccordingToPlan.setToNaN();
   }

   @Override
   public void doAction(double timeInState)
   {
      super.doAction(timeInState);

      boolean requestSwingSpeedUp = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

      if (walkingMessageHandler.hasRequestedFootstepAdjustment())
      {
         boolean footstepHasBeenAdjusted = walkingMessageHandler.pollRequestedFootstepAdjustment(nextFootstep);

         if (footstepHasBeenAdjusted)
         {
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

            balanceManager.updateCurrentICPPlan();
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

            feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

            walkingMessageHandler.reportWalkingAbortRequested();
            walkingMessageHandler.clearFootsteps();

            double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
            double swingDuration = footstepTiming.getSwingTime();
            double transferDuration = footstepTiming.getTransferTime();

            double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
            double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();

            balanceManager.clearICPPlan();
            balanceManager.setFinalTransferTime(finalTransferTime);
            balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
            balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
            balanceManager.addFootstepToPlan(nextFootstep, footstepTiming, footstepShiftFraction);
            balanceManager.setICPPlanSupportSide(supportSide);
            balanceManager.initializeICPPlanForSingleSupport(swingDuration, transferDuration, finalTransferTime);
         }
      }
      else
      {
         boolean footstepIsBeingAdjusted = balanceManager.checkAndUpdateFootstepFromICPOptimization(nextFootstep);

         if (footstepIsBeingAdjusted)
         {
            requestSwingSpeedUp = true;
            walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
            failureDetectionControlModule.setNextFootstep(nextFootstep);
            updateFootstepParameters();

            feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

            balanceManager.updateCurrentICPPlan();
            //legConfigurationManager.prepareForLegBracing(swingSide);
         }

         // if the footstep was adjusted, shift the CoM plan, if there is one.
         walkingMessageHandler.setPlanOffsetFromAdjustment(balanceManager.getEffectiveICPAdjustment());
      }

      if (requestSwingSpeedUp)
      {
         double swingTimeRemaining = requestSwingSpeedUpIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }
      boolean feetAreWellPositioned = legConfigurationManager.areFeetWellPositionedForCollapse(swingSide.getOppositeSide(),
                                                                                               nextFootstep.getSoleReferenceFrame());

      if (timeInState > fractionOfSwingToStraightenSwingLeg.getDoubleValue() * swingTime)
      {
         legConfigurationManager.straightenLegDuringSwing(swingSide);
      }
      if (timeInState > fractionOfSwingToCollapseStanceLeg.getDoubleValue() * swingTime && !legConfigurationManager.isLegCollapsed(supportSide)
            && feetAreWellPositioned)
      {
         legConfigurationManager.collapseLegDuringSwing(swingSide.getOppositeSide());
      }

      walkingMessageHandler.clearFootTrajectory();

      switchToToeOffIfPossible(supportSide);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (super.isDone(timeInState))
      {
         return true;
      }

      return finishSingleSupportWhenICPPlannerIsDone.getBooleanValue() && balanceManager.isICPPlanDone();
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      double defaultSwingTime = walkingMessageHandler.getDefaultSwingTime();
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         swingTime = defaultSwingTime;
         footstepTiming.setTimings(swingTime, defaultTransferTime);
         balanceManager.packFootstepForRecoveringFromDisturbance(swingSide, defaultSwingTime, nextFootstep);
         nextFootstep.setTrajectoryType(TrajectoryType.DEFAULT);
         walkingMessageHandler.reportWalkingAbortRequested();
         walkingMessageHandler.clearFootsteps();
      }
      else
      {
         swingTime = walkingMessageHandler.getNextSwingTime();
         walkingMessageHandler.poll(nextFootstep, footstepTiming, footstepShiftFraction);
      }

      /** 1/08/2018 RJG this has to be done before calling #updateFootstepParameters() to make sure the contact points are up to date */
      feetManager.setContactStateForSwing(swingSide);

      updateFootstepParameters();

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringSwing.getValue());
      balanceManager.setNextFootstep(nextFootstep);
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming, footstepShiftFraction);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      boolean isLastStep = stepsToAdd == 0;
      for (int i = 0; i < stepsToAdd; i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps[i]);
         walkingMessageHandler.peekTiming(i, footstepTimings[i]);
         walkingMessageHandler.peekShiftFraction(i, footstepShiftFractions[i]);
         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i], footstepShiftFractions[i]);
      }

      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport(footstepTiming.getSwingTime(), footstepTiming.getTransferTime(), finalTransferTime);

      if (balanceManager.wasTimingAdjustedForReachability())
      {
         double currentTransferDuration = balanceManager.getCurrentTransferDurationAdjustedForReachability();
         double currentSwingDuration = balanceManager.getCurrentSwingDurationAdjustedForReachability();
         double nextTransferDuration = balanceManager.getNextTransferDurationAdjustedForReachability();

         swingTime = currentSwingDuration;
         footstepTiming.setTimings(currentSwingDuration, currentTransferDuration);

         if (isLastStep)
         {
            balanceManager.setFinalTransferTime(nextTransferDuration);
         }
         else
         {
            double nextSwingTime = footstepTimings[0].getSwingTime();
            footstepTimings[0].setTimings(nextSwingTime, nextTransferDuration);
            walkingMessageHandler.adjustTiming(nextSwingTime, nextTransferDuration);
         }
      }

      if (balanceManager.isRecoveringFromDoubleSupportFall())
      {
         balanceManager.updateCurrentICPPlan();
         balanceManager.requestICPPlannerToHoldCurrentCoMInNextDoubleSupport();
      }

      feetManager.requestSwing(swingSide, nextFootstep, swingTime);

      if (feetManager.adjustHeightIfNeeded(nextFootstep))
      {
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);
      }

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
   public void onExit()
   {
      super.onExit();

      balanceManager.minimizeAngularMomentumRateZ(false);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);

      touchdownErrorCompensator.registerDesiredFootstepPosition(swingSide, desiredFootPoseInWorld.getPosition());

      walkingMessageHandler.reportFootstepCompleted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld);
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);

      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(nextFootstep.getRobotSide());
      tempOrientation.setIncludingFrame(nextFootstep.getFootstepPose().getOrientation());
      tempOrientation.changeFrame(soleZUpFrame);
      double pitch = tempOrientation.getPitch();

      if (doManualTouchdown())
      {
         // Get the initial condition from the swing trajectory
         FrameSE3TrajectoryPoint lastWaypoint = nextFootstep.getSwingTrajectory().get(nextFootstep.getSwingTrajectory().size() - 1);
         tempOrientation.setIncludingFrame(lastWaypoint.getOrientation());
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(lastWaypoint.getAngularVelocity());
         tempAngularVelocity.changeFrame(soleZUpFrame); // The y component is equivalent to the pitch rate since the yaw and roll rate are 0.0
      }
      else
      {
         // Get the initial condition from the robot state
         MovingReferenceFrame soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(nextFootstep.getRobotSide());
         tempOrientation.setToZero(soleFrame);
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(soleFrame.getTwistOfFrame().getAngularPart());
         tempAngularVelocity.changeFrame(soleZUpFrame);
      }
      double initialPitch = tempOrientation.getPitch();
      double initialPitchVelocity = tempAngularVelocity.getY();
      feetManager.touchDown(nextFootstep.getRobotSide(), initialPitch, initialPitchVelocity, pitch, footstepTiming.getTouchdownDuration());

      setYoVariablesToNaN();
   }

   private boolean doManualTouchdown()
   {
      return nextFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(nextFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
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
    * Request the swing trajectory to speed up using
    * {@link us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface#estimateTimeRemainingForStateUnderDisturbance(FramePoint2D)}.
    * It is clamped w.r.t. to
    * {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    *
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
      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodies.
      controllerToolbox.updateContactPointsForUpcomingFootstep(nextFootstep);
      controllerToolbox.updateBipedSupportPolygons();

      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setUpcomingFootstep(nextFootstep);
      pelvisOrientationManager.updateTrajectoryFromFootstep(); // fixme this shouldn't be called when the footstep is updated

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep,
                                                                                                                                                swingSide);
      transferToAndNextFootstepsData.setTransferFromDesiredFootstep(walkingMessageHandler.getLastDesiredFootstep(supportSide));
      double extraToeOffHeight = 0.0;
      if (feetManager.canDoSingleSupportToeOff(nextFootstep, swingSide))
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);

      FixedFramePoint3DBasics stanceFootPosition = walkingMessageHandler.getFootstepAtCurrentLocation(swingSide.getOppositeSide()).getFootstepPose().getPosition();
      FixedFramePoint3DBasics touchdownPosition = nextFootstep.getFootstepPose().getPosition();
      double swingTime = footstepTiming.getSwingTime(); // TODO: Should be swing time remaining for step adjustments.
      comHeightManager.step(stanceFootPosition, touchdownPosition, swingTime, swingSide, extraToeOffHeight);
   }

   @Override
   protected boolean hasMinimumTimePassed(double timeInState)
   {
      double minimumSwingTime;
      if (balanceManager.isRecoveringFromDoubleSupportFall())
         minimumSwingTime = 0.15;
      else
         minimumSwingTime = swingTime * minimumSwingFraction.getDoubleValue();

      return timeInState > minimumSwingTime;
   }
}