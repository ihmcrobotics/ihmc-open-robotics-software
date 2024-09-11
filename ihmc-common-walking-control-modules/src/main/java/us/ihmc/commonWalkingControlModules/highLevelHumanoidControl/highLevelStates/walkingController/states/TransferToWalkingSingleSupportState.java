package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferToWalkingSingleSupportState extends TransferState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int numberOfFootstepsToConsider;
   private final Footstep[] footsteps;
   private final FootstepTiming[] footstepTimings;

   private final DoubleProvider minimumTransferTime;

   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);
   private final YoBoolean resubmitStepsInTransferEveryTick = new YoBoolean("resubmitStepsInTransferEveryTick", registry);

   private final YoDouble originalTransferTime = new YoDouble("OriginalTransferTime", registry);
   private final BooleanProvider minimizeAngularMomentumRateZDuringTransfer;
   private final DoubleProvider icpErrorThresholdForSlowTransfer;
   private final DoubleProvider minimumSlowTransferDuration;

   private final FramePose3D actualFootPositionInWorld = new FramePose3D();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   private final TouchdownErrorCompensator touchdownErrorCompensator;

   // This flag indicates whether or not its the first tick in the transfer state. This is used to avoid double-computing some of the calls.
   private boolean firstTickInState = true;

   public TransferToWalkingSingleSupportState(WalkingStateEnum stateEnum,
                                              WalkingMessageHandler walkingMessageHandler,
                                              TouchdownErrorCompensator touchdownErrorCompensator,
                                              HighLevelHumanoidControllerToolbox controllerToolbox,
                                              HighLevelControlManagerFactory managerFactory,
                                              WalkingControllerParameters walkingControllerParameters,
                                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                                              DoubleProvider minimumTransferTime,
                                              DoubleProvider unloadFraction,
                                              DoubleProvider rhoMin,
                                              YoRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, unloadFraction, rhoMin, parentRegistry);

      this.minimumTransferTime = minimumTransferTime;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      minimizeAngularMomentumRateZDuringTransfer = new BooleanParameter("minimizeAngularMomentumRateZDuringTransfer",
                                                                        registry,
                                                                        walkingControllerParameters.minimizeAngularMomentumRateZDuringTransfer());
      icpErrorThresholdForSlowTransfer = ParameterProvider.getOrCreateParameter(parentRegistry.getName(),
                                                                                getClass().getSimpleName(),
                                                                                "icpErrorThresholdForSlowTransfer",
                                                                                registry,
                                                                                walkingControllerParameters.getInitialICPErrorToSlowDownTransfer());
      minimumSlowTransferDuration = ParameterProvider.getOrCreateParameter(parentRegistry.getName(),
                                                                           getClass().getSimpleName(),
                                                                           "minimumSlowTransferDuration",
                                                                           registry,
                                                                           walkingControllerParameters.getMinimumSlowTransferDuration());
      resubmitStepsInTransferEveryTick.set(walkingControllerParameters.resubmitStepsInTransferEveryTick());

      numberOfFootstepsToConsider = balanceManager.getMaxNumberOfStepsToConsider();
      footsteps = Footstep.createFootsteps(numberOfFootstepsToConsider);
      footstepTimings = FootstepTiming.createTimings(numberOfFootstepsToConsider);
   }

   @Override
   protected void updateICPPlan()
   {
      super.updateICPPlan();

      // This needs to check `TO_STANDING` as well as messages could be received on the very first controller tick at which point
      // the robot is not in the standing state but not yet walking either.
      if (getPreviousWalkingStateEnum() == WalkingStateEnum.STANDING || getPreviousWalkingStateEnum() == WalkingStateEnum.TO_STANDING)
      {
         walkingMessageHandler.reportWalkingStarted();
      }

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      balanceManager.setFinalTransferTime(finalTransferTime);

      int stepsToAdd = Math.min(numberOfFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      if (stepsToAdd < 1)
      {
         throw new RuntimeException("Can not go to walking single support if there are no upcoming footsteps.");
      }
      for (int i = 0; i < stepsToAdd; i++)
      {
         Footstep footstep = footsteps[i];
         FootstepTiming timing = footstepTimings[i];
         walkingMessageHandler.peekFootstep(i, footstep);
         walkingMessageHandler.peekTiming(i, timing);

         if (i == 0)
         {
            adjustTiming(timing);
            walkingMessageHandler.adjustTiming(timing.getSwingTime(), timing.getTransferTime());
         }

         balanceManager.addFootstepToPlan(footstep, timing);
      }

      FootstepTiming firstTiming = footstepTimings[0];
      currentTransferDuration.set(firstTiming.getTransferTime());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransfer();
   }

   @Override
   public void doAction(double timeInState)
   {
      if (resubmitStepsInTransferEveryTick.getBooleanValue()
            && balanceManager.getNumberOfStepsBeingConsidered() < walkingMessageHandler.getCurrentNumberOfFootsteps())
      {
         int stepsToAdd = Math.min(numberOfFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
         for (int i = balanceManager.getNumberOfStepsBeingConsidered() - 1; i < stepsToAdd; i++)
         {
            Footstep footstep = footsteps[i];
            FootstepTiming timing = footstepTimings[i];

            walkingMessageHandler.peekFootstep(i, footstep);
            walkingMessageHandler.peekTiming(i, timing);

            balanceManager.addFootstepToPlan(footstep, timing);
         }
      }

      RobotSide swingSide = transferToSide.getOppositeSide();
      if (!firstTickInState)
         feetManager.updateSwingTrajectoryPreview(swingSide);
      balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));
      balanceManager.computeICPPlan();
      updateWalkingTrajectoryPath();

      if (!doManualLiftOff())
      {
         if (switchToToeOffIfPossible())
            feetManager.initializeSwingTrajectoryPreview(swingSide, footsteps[0], footstepTimings[0].getSwingTime());
      }

      super.doAction(timeInState);

      double transferDuration = currentTransferDuration.getDoubleValue();

      updateFootPlanOffset();

      double toeOffDuration = footstepTimings[0].getLiftoffDuration();
      if (doManualLiftOff() && transferDuration - timeInState < toeOffDuration)
      {
         Footstep upcomingFootstep = footsteps[0];
         FrameSE3TrajectoryPoint firstWaypoint = upcomingFootstep.getSwingTrajectory().get(0);
         MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide.getOppositeSide());
         tempOrientation.setIncludingFrame(firstWaypoint.getOrientation());
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(firstWaypoint.getAngularVelocity());
         tempAngularVelocity.changeFrame(soleZUpFrame); // The y component is equivalent to the pitch rate since the yaw and roll rate are 0.0
         feetManager.liftOff(transferToSide.getOppositeSide(), tempOrientation.getPitch(), tempAngularVelocity.getY(), toeOffDuration);
      }

      firstTickInState = false;
   }

   private void updateWalkingTrajectoryPath()
   {
      walkingTrajectoryPath.clearFootsteps();
      walkingTrajectoryPath.addFootsteps(walkingMessageHandler);
      walkingTrajectoryPath.updateTrajectory(feetManager.getCurrentConstraintType(RobotSide.LEFT), feetManager.getCurrentConstraintType(RobotSide.RIGHT));
   }

   private boolean doManualLiftOff()
   {
      Footstep upcomingFootstep = footsteps[0];
      return upcomingFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(upcomingFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
   }

   @Override
   public void onEntry()
   {
      firstTickInState = true;

      if (balanceManager.getICPErrorMagnitude() > icpErrorThresholdForSlowTransfer.getValue())
      {
         walkingMessageHandler.peekTiming(0, footstepTimings[0]);
         double transferDuration = Math.max(minimumSlowTransferDuration.getValue(), 2.0 * footstepTimings[0].getTransferTime());
         walkingMessageHandler.setUpcomingFootstepTransferDuration(transferDuration);
      }

      super.onEntry();

      feetManager.initializeSwingTrajectoryPreview(transferToSide.getOppositeSide(), footsteps[0], footstepTimings[0].getSwingTime());
      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringTransfer.getValue());

      updateFootPlanOffset();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return super.isDone(timeInState) || feetManager.isFootToeingOffSlipping(transferToSide.getOppositeSide());
   }

   @Override
   public void onExit(double timeInState)
   {
      super.onExit(timeInState);

      touchdownErrorCompensator.commitToFootTouchdownError(transferToSide);
      touchdownErrorCompensator.clear();
      firstTickInState = true;
      balanceManager.minimizeAngularMomentumRateZ(false);
   }

   /**
    * This method checks if the upcoming step has a desired absolute start time. If that is the case
    * the transfer time is adjusted such that the swing starts at the correct time.
    */
   private void adjustTiming(FootstepTiming stepTiming)
   {
      if (!stepTiming.hasAbsoluteTime())
      {
         originalTransferTime.setToNaN();
         return;
      }

      double originalSwingTime = stepTiming.getSwingTime();
      double originalTransferTime = stepTiming.getTransferTime();
      this.originalTransferTime.set(originalTransferTime);

      double currentTime = controllerToolbox.getYoTime().getDoubleValue();
      double timeInFootstepPlan = currentTime - stepTiming.getExecutionStartTime();
      double adjustedTransferTime = stepTiming.getSwingStartTime() - timeInFootstepPlan;

      // make sure transfer does not get too short
      adjustedTransferTime = Math.max(adjustedTransferTime, minimumTransferTime.getValue());

      // GW TODO - possible improvement:
      // If the adjustment is capped by the minimum transfer time adjust also the upcoming transfer times here. That
      // would make the ICP plan for the upcoming steps more accurate. However, if the given original transfer times
      // are correctly set this might be a minimal improvement that makes step timing more complicated and difficult
      // to debug. If we have big adjustments a lot we should revisit this.

      // keep swing times and only adjust transfers for now
      stepTiming.setTimings(originalSwingTime, adjustedTransferTime);
   }

   private void updateFootPlanOffset()
   {
      WalkingStateEnum previousStateEnum = getPreviousWalkingStateEnum();
      if (previousStateEnum == null)
         return;

      RobotSide previousSupportSide = previousStateEnum.getSupportSide();
      if (previousSupportSide == null)
         return;

      RobotSide previousSwingSide = previousSupportSide.getOppositeSide();
      if (touchdownErrorCompensator.planShouldBeOffsetFromStep(previousSwingSide) && touchdownErrorCompensator.isFootPositionTrusted(previousSwingSide))
      {
         actualFootPositionInWorld.setToZero(controllerToolbox.getReferenceFrames().getSoleFrame(previousSwingSide));
         actualFootPositionInWorld.changeFrame(worldFrame);

         touchdownErrorCompensator.updateFootTouchdownError(previousSwingSide, actualFootPositionInWorld);
      }
   }
}