package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferToWalkingSingleSupportState extends TransferState
{
   private static final int numberOfFootstepsToConsider = 3;
   private final Footstep[] footsteps = Footstep.createFootsteps(numberOfFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(numberOfFootstepsToConsider);

   private final DoubleProvider minimumTransferTime;

   private final LegConfigurationManager legConfigurationManager;
   private final YoDouble fractionOfTransferToCollapseLeg = new YoDouble("fractionOfTransferToCollapseLeg", registry);
   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   private final YoDouble originalTransferTime = new YoDouble("OriginalTransferTime", registry);
   private final BooleanProvider minimizeAngularMomentumRateZDuringTransfer;


   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public TransferToWalkingSingleSupportState(WalkingStateEnum stateEnum, WalkingMessageHandler walkingMessageHandler,
                                              HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                                              WalkingControllerParameters walkingControllerParameters,
                                              WalkingFailureDetectionControlModule failureDetectionControlModule, DoubleProvider minimumTransferTime,
                                              DoubleProvider unloadFraction, DoubleProvider rhoMin, YoVariableRegistry parentRegistry)
   {
      super(stateEnum, walkingControllerParameters, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, unloadFraction,
            rhoMin, parentRegistry);

      this.minimumTransferTime = minimumTransferTime;

      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      fractionOfTransferToCollapseLeg.set(walkingControllerParameters.getLegConfigurationParameters().getFractionOfTransferToCollapseLeg());
      minimizeAngularMomentumRateZDuringTransfer = new BooleanParameter("minimizeAngularMomentumRateZDuringTransfer", registry,
                                                                        walkingControllerParameters.minimizeAngularMomentumRateZDuringTransfer());
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

      if (isInitialTransfer())
      {
         pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
      }
      else
      {
         // In middle of walking or leaving foot pose, pelvis is good leave it like that.
         pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
      }

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      walkingMessageHandler.requestPlanarRegions();
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

      balanceManager.setICPPlanTransferToSide(transferToSide);
      FootstepTiming firstTiming = footstepTimings[0];
      currentTransferDuration.set(firstTiming.getTransferTime());
      balanceManager.initializeICPPlanForTransfer(firstTiming.getSwingTime(), firstTiming.getTransferTime(), finalTransferTime);

      if (balanceManager.wasTimingAdjustedForReachability())
      {
         double currentTransferDuration = balanceManager.getCurrentTransferDurationAdjustedForReachability();
         double currentSwingDuration = balanceManager.getCurrentSwingDurationAdjustedForReachability();
         firstTiming.setTimings(currentSwingDuration, currentTransferDuration);
      }

      pelvisOrientationManager.setUpcomingFootstep(footsteps[0]);
      pelvisOrientationManager.initializeTransfer(transferToSide, firstTiming.getTransferTime(), firstTiming.getSwingTime());

      legConfigurationManager.beginStraightening(transferToSide);
      legConfigurationManager.setFullyExtendLeg(transferToSide, false);
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!doManualLiftOff())
         switchToToeOffIfPossible();

      super.doAction(timeInState);

      double transferDuration = currentTransferDuration.getDoubleValue();
      boolean pastMinimumTime = timeInState > fractionOfTransferToCollapseLeg.getDoubleValue() * transferDuration;
      boolean isFootWellPosition = legConfigurationManager.areFeetWellPositionedForCollapse(transferToSide.getOppositeSide());
      if (pastMinimumTime && isFootWellPosition && !legConfigurationManager.isLegCollapsed(transferToSide.getOppositeSide()))
      {
         legConfigurationManager.collapseLegDuringTransfer(transferToSide);
      }

      double toeOffDuration = footstepTimings[0].getLiftoffDuration();
      if (doManualLiftOff() && transferDuration - timeInState < toeOffDuration)
      {
         Footstep upcomingFootstep = footsteps[0];
         FrameSE3TrajectoryPoint firstWaypoint = upcomingFootstep.getSwingTrajectory().get(0);
         tempOrientation.setIncludingFrame(firstWaypoint.getOrientation());
         tempOrientation.changeFrame(controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide.getOppositeSide()));
         feetManager.liftOff(transferToSide.getOppositeSide(), tempOrientation.getPitch(), toeOffDuration);
      }
   }

   private boolean doManualLiftOff()
   {
      Footstep upcomingFootstep = footsteps[0];
      return upcomingFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(upcomingFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringTransfer.getValue());
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
}