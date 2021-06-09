package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferToRecoveringSingleSupportState extends PushRecoveryTransferState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int numberOfFootstepsToConsider;
   private final Footstep footsteps;
   private final FootstepTiming footstepTimings;

   private final DoubleProvider minimumTransferTime;
   private final DoubleProvider minimumSwingTime;

   private final LegConfigurationManager legConfigurationManager;
   private final YoDouble fractionOfTransferToCollapseLeg = new YoDouble("fractionOfTransferToCollapseLeg", registry);
   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   private final YoDouble originalTransferTime = new YoDouble("OriginalTransferTime", registry);
   private final BooleanProvider minimizeAngularMomentumRateZDuringTransfer;

   private final FramePoint3D actualFootPositionInWorld = new FramePoint3D();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   private final TouchdownErrorCompensator touchdownErrorCompensator;

   public TransferToRecoveringSingleSupportState(PushRecoveryStateEnum stateEnum,
                                                 WalkingMessageHandler walkingMessageHandler,
                                                 TouchdownErrorCompensator touchdownErrorCompensator,
                                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                                 HighLevelControlManagerFactory managerFactory,
                                                 PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                                 WalkingFailureDetectionControlModule failureDetectionControlModule,
                                                 DoubleProvider minimumTransferTime,
                                                 DoubleProvider minimumSwingTime,
                                                 DoubleProvider unloadFraction,
                                                 DoubleProvider rhoMin,
                                                 YoRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, unloadFraction,
            rhoMin, parentRegistry);

      this.minimumTransferTime = minimumTransferTime;
      this.minimumSwingTime = minimumSwingTime;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      fractionOfTransferToCollapseLeg.set(pushRecoveryControllerParameters.getLegConfigurationParameters().getFractionOfTransferToCollapseLeg());
      minimizeAngularMomentumRateZDuringTransfer = new BooleanParameter("minimizeAngularMomentumRateZDuringTransfer", registry,
              pushRecoveryControllerParameters.minimizeAngularMomentumRateZDuringTransfer());

      numberOfFootstepsToConsider = 1;    //TODO move to pushRecoveryControllerParameters
      footsteps = new Footstep();
      footstepTimings = new FootstepTiming();
   }

   @Override
   protected void updateICPPlan()
   {
      super.updateICPPlan();

      // In middle of walking or leaving foot pose, pelvis is good leave it like that.
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      walkingMessageHandler.requestPlanarRegions();
      balanceManager.setFinalTransferTime(finalTransferTime);

      currentTransferDuration.set(minimumTransferTime.getValue());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransferToRecovery();

//      pelvisOrientationManager.setUpcomingFootstep(footsteps);
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
      pelvisOrientationManager.initializeTransfer(transferToSide, minimumTransferTime.getValue(), minimumSwingTime.getValue());

      legConfigurationManager.beginStraightening(transferToSide);
      legConfigurationManager.setFullyExtendLeg(transferToSide, false);
   }

   @Override
   public void doAction(double timeInState)
   {
      RobotSide swingSide = transferToSide.getOppositeSide();
      feetManager.updateSwingTrajectoryPreview(swingSide);
      balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));
      balanceManager.computeICPPlan();

      if (!doManualLiftOff())
      {
         if (switchToToeOffIfPossible())
            feetManager.initializeSwingTrajectoryPreview(swingSide, footsteps, footstepTimings.getSwingTime());
      }

      super.doAction(timeInState);

      double transferDuration = currentTransferDuration.getDoubleValue();
      boolean pastMinimumTime = timeInState > fractionOfTransferToCollapseLeg.getDoubleValue() * transferDuration;
      boolean isFootWellPosition = legConfigurationManager.areFeetWellPositionedForCollapse(transferToSide.getOppositeSide());
      if (pastMinimumTime && isFootWellPosition && !legConfigurationManager.isLegCollapsed(transferToSide.getOppositeSide()))
      {
         legConfigurationManager.collapseLegDuringTransfer(transferToSide);
      }

      updateFootPlanOffset();

      double toeOffDuration = footstepTimings.getLiftoffDuration();
      if (doManualLiftOff() && transferDuration - timeInState < toeOffDuration)
      {
         Footstep upcomingFootstep = footsteps;
         FrameSE3TrajectoryPoint firstWaypoint = upcomingFootstep.getSwingTrajectory().get(0);
         MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide.getOppositeSide());
         tempOrientation.setIncludingFrame(firstWaypoint.getOrientation());
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(firstWaypoint.getAngularVelocity());
         tempAngularVelocity.changeFrame(soleZUpFrame); // The y component is equivalent to the pitch rate since the yaw and roll rate are 0.0
         feetManager.liftOff(transferToSide.getOppositeSide(), tempOrientation.getPitch(), tempAngularVelocity.getY(), toeOffDuration);
      }
   }

   private boolean doManualLiftOff()
   {
      Footstep upcomingFootstep = footsteps;
      return upcomingFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(upcomingFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      feetManager.initializeSwingTrajectoryPreview(transferToSide.getOppositeSide(), footsteps, footstepTimings.getSwingTime());
      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringTransfer.getValue());

      updateFootPlanOffset();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return super.isDone(timeInState) || feetManager.isFootToeingOffSlipping(transferToSide.getOppositeSide());
   }

   @Override
   public void onExit()
   {
      super.onExit();

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
      PushRecoveryStateEnum previousStateEnum = getPreviousWalkingStateEnum();
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

         touchdownErrorCompensator.addOffsetVectorFromTouchdownError(previousSwingSide, actualFootPositionInWorld);
      }
   }
}