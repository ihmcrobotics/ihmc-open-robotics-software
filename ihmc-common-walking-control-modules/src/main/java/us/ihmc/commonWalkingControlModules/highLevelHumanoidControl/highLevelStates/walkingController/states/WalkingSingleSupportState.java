package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.JointOfflineManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegionsList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class WalkingSingleSupportState extends SingleSupportState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int additionalFootstepsToConsider;
   private final Footstep nextFootstep = new Footstep();
   private final Footstep nextNextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private double swingTime;

   private final Footstep[] footsteps;
   private final FootstepTiming[] footstepTimings;

   private final FramePose3D actualFootPoseInWorld = new FramePose3D(worldFrame);
   private final FramePose3D desiredFootPoseInWorld = new FramePose3D(worldFrame);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final JointOfflineManager jointOfflineManager;

   private final YoBoolean finishSingleSupportWhenICPPlannerIsDone = new YoBoolean("finishSingleSupportWhenICPPlannerIsDone", registry);
   private final YoBoolean resubmitStepsInSwingEveryTick = new YoBoolean("resubmitStepsInSwingEveryTick", registry);

   private final BooleanProvider minimizeAngularMomentumRateZDuringSwing;

   private final DoubleProvider timeOverrunToInitializeFreeFall;

   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   private final TouchdownErrorCompensator touchdownErrorCompensator;
   private final StepConstraintRegionsList stepConstraints = new StepConstraintRegionsList();

   public WalkingSingleSupportState(WalkingStateEnum stateEnum,
                                    WalkingMessageHandler walkingMessageHandler,
                                    TouchdownErrorCompensator touchdownErrorCompensator,
                                    HighLevelHumanoidControllerToolbox controllerToolbox,
                                    HighLevelControlManagerFactory managerFactory,
                                    WalkingControllerParameters walkingControllerParameters,
                                    WalkingFailureDetectionControlModule failureDetectionControlModule,
                                    YoRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, parentRegistry);

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      jointOfflineManager = managerFactory.getOrCreateJointOfflineManager();

      finishSingleSupportWhenICPPlannerIsDone.set(walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone());
      minimizeAngularMomentumRateZDuringSwing = new BooleanParameter("minimizeAngularMomentumRateZDuringSwing",
                                                                     registry,
                                                                     walkingControllerParameters.minimizeAngularMomentumRateZDuringSwing());

      timeOverrunToInitializeFreeFall = ParameterProvider.getOrCreateParameter(parentRegistry.getName(),
                                                                               getClass().getSimpleName(),
                                                                               "swingTimeOverrunToInitializeFreeFall",
                                                                               registry,
                                                                               walkingControllerParameters.getSwingTimeOverrunToInitializeFreeFall());
      resubmitStepsInSwingEveryTick.set(walkingControllerParameters.resubmitStepsInSwingEveryTick());

      additionalFootstepsToConsider = balanceManager.getMaxNumberOfStepsToConsider();
      footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
      footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);
   }

   @Override
   public void doAction(double timeInState)
   {
      if (resubmitStepsInSwingEveryTick.getBooleanValue())
      {
         balanceManager.clearICPPlan();
         balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);

         int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
         for (int i = 0; i < stepsToAdd; i++)
         {
            walkingMessageHandler.peekFootstep(i, footsteps[i]);
            walkingMessageHandler.peekTiming(i, footstepTimings[i]);
            balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i]);
         }
      }

      balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));
      balanceManager.computeICPPlan();
      updateWalkingTrajectoryPath();

      // call this here, too, so that the time in state is updated properly for all the swing speed up stuff, so it doesn't get out of sequence. This is
      // normally called in the WalkingHighLevelHumanoidController.balanceManager#compute
      balanceManager.updateTimeInState();
      boolean requestSwingSpeedUp = balanceManager.shouldAdjustTimeFromTrackingError();

      // in the first tick of single support, the state machine for the feet manager likely hasn't transitioned to swing yet. It's also possible this has been
      // delayed for another reason. So we should check before updating step adjustment
      boolean footstepIsBeingAdjusted = false;
      if (!feetManager.getCurrentConstraintType(swingSide).isLoadBearing())
         footstepIsBeingAdjusted = balanceManager.checkAndUpdateStepAdjustment(nextFootstep);
      
      if (footstepIsBeingAdjusted)
      {
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
         updateFootstepParameters();

         feetManager.adjustSwingTrajectory(swingSide,
                                           nextFootstep,
                                           balanceManager.getFinalDesiredCoMVelocity(),
                                           balanceManager.getFinalDesiredCoMAcceleration(),
                                           swingTime);

         balanceManager.adjustFootstepInCoPPlan(nextFootstep);
         // FIXME I don't need to be computing this again
         balanceManager.computeICPPlan();

         updateHeightManager();
      }

      // TODO look at setting upcoming contact state here based on joint going offline

      if (requestSwingSpeedUp || footstepIsBeingAdjusted)
      {
         double swingTimeRemaining = requestSwingSpeedUpInFeetManagerIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }

      if (timeInState > swingTime + timeOverrunToInitializeFreeFall.getValue())
      {
         // TODO Not sure if the transition duration should be fixed or a scale of the swing time. Need to be extracted. 
         comHeightManager.initializeTransitionToFall(swingTime / 6.0);
      }

      walkingMessageHandler.clearFlamingoCommands();

      switchToToeOffIfPossible(supportSide);
   }

   private void updateWalkingTrajectoryPath()
   {
      walkingTrajectoryPath.clearFootsteps();
      walkingTrajectoryPath.addFootstep(nextFootstep, footstepTiming);
      walkingTrajectoryPath.addFootsteps(walkingMessageHandler);
      walkingTrajectoryPath.updateTrajectory(feetManager.getCurrentConstraintType(RobotSide.LEFT), feetManager.getCurrentConstraintType(RobotSide.RIGHT));
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

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      swingTime = walkingMessageHandler.getNextSwingTime();
      walkingMessageHandler.poll(nextFootstep, footstepTiming);
      if (walkingMessageHandler.getCurrentNumberOfFootsteps() > 0)
         walkingMessageHandler.peekFootstep(0, nextNextFootstep);

      desiredFootPoseInWorld.set(nextFootstep.getFootstepPose());
      desiredFootPoseInWorld.changeFrame(worldFrame);

      /**
       * 1/08/2018 RJG this has to be done before calling #updateFootstepParameters() to make sure the
       * contact points are up to date
       */
      feetManager.setContactStateForSwing(swingSide);

      updateFootstepParameters();

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringSwing.getValue());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      for (int i = 0; i < stepsToAdd; i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps[i]);
         walkingMessageHandler.peekTiming(i, footstepTimings[i]);
         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i]);
      }

      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport();

      /** This has to be called after calling initialize ICP Plan, as that resets the step constraints **/
      walkingMessageHandler.pollStepConstraints(stepConstraints);
      balanceManager.setCurrentStepConstraints(stepConstraints);

      updateHeightManager();

      feetManager.requestSwing(swingSide,
                               nextFootstep,
                               swingTime,
                               balanceManager.getFinalDesiredCoMVelocity(),
                               balanceManager.getFinalDesiredCoMAcceleration());

      if (feetManager.adjustHeightIfNeeded(nextFootstep))
      {
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         feetManager.adjustSwingTrajectory(swingSide,
                                           nextFootstep,
                                           swingTime);
      }

      balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));

      pelvisOrientationManager.initializeSwing();


      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepStarted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime, nextFootstep.getSequenceID());
   }

   @Override
   public void onExit(double timeInState)
   {
      super.onExit(timeInState);

      balanceManager.minimizeAngularMomentumRateZ(false);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);

      touchdownErrorCompensator.registerCompletedFootstep(swingSide, desiredFootPoseInWorld, nextFootstep);

      walkingMessageHandler.reportFootstepCompleted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime, nextFootstep.getSequenceID());
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
   }

   private boolean doManualTouchdown()
   {
      return nextFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(nextFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
   }

   private final FramePoint2D desiredCoP = new FramePoint2D(worldFrame);
   private final FramePoint2D currentICP = new FramePoint2D(worldFrame);

   public void switchToToeOffIfPossible(RobotSide supportSide)
   {
      currentICP.setIncludingFrame(balanceManager.getCapturePoint());

      controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(supportSide), desiredCoP);

      FramePoint3DReadOnly supportFootExitCMP = balanceManager.getFirstExitCMPForToeOff(false);

      feetManager.updateToeOffStatusSingleSupport(nextFootstep,
                                                  supportFootExitCMP,
                                                  balanceManager.getDesiredCMP(),
                                                  balanceManager.getDesiredICP(),
                                                  currentICP);

      if (feetManager.okForPointToeOff(true))
         feetManager.requestPointToeOff(supportSide, supportFootExitCMP, desiredCoP);
      else if (feetManager.okForLineToeOff(true))
         feetManager.requestLineToeOff(supportSide, supportFootExitCMP, desiredCoP);
   }

   /**
    * Request the swing trajectory. It is clamped w.r.t. to
    * {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    *
    * @return the current swing time remaining for the swing foot trajectory
    */
   private double requestSwingSpeedUpInFeetManagerIfNeeded()
   {
      double remainingSwingTimeAccordingToPlan = balanceManager.getTimeRemainingInCurrentState();
      double adjustedRemainingTime = Math.max(0.0,
                                              balanceManager.getAdjustedTimeRemainingInCurrentSupportSequence()
                                                    - balanceManager.getExtraTimeAdjustmentForSwing());

      if (adjustedRemainingTime > 1.0e-3)
      {
         double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan / adjustedRemainingTime;
         return feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
      }
      else if (remainingSwingTimeAccordingToPlan > 1.0e-3)
      {
         return feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
      }
      return remainingSwingTimeAccordingToPlan;
   }

   private void updateFootstepParameters()
   {
      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodies.
      controllerToolbox.updateContactPointsForUpcomingFootstep(nextFootstep);
      controllerToolbox.updateBipedSupportPolygons();

      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setUpcomingFootstep(nextFootstep);
      pelvisOrientationManager.updateTrajectoryFromFootstep(); // fixme this shouldn't be called when the footstep is updated

   }

   private void updateHeightManager()
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep,
                                                                                                                                                swingSide);
      transferToAndNextFootstepsData.setComAtEndOfState(balanceManager.getFinalDesiredCoMPosition());
      double extraToeOffHeight = 0.0;
      if (feetManager.canDoSingleSupportToeOff(nextFootstep.getFootstepPose(), swingSide) && feetManager.isSteppingUp())
         extraToeOffHeight = feetManager.getExtraCoMMaxHeightWithToes();
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   @Override
   protected boolean hasMinimumTimePassed(double timeInState)
   {
      return feetManager.getFractionThroughSwing(swingSide) > minimumSwingFraction.getDoubleValue();
   }
}