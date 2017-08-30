package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotSide;

public class TransferToWalkingSingleSupportState extends TransferState
{
   private final YoDouble minimumTransferTime = new YoDouble("minimumTransferTime", registry);

   private final LegConfigurationManager legConfigurationManager;
   private final YoDouble fractionOfTransferToCollapseLeg = new YoDouble("fractionOfTransferToCollapseLeg", registry);

   public TransferToWalkingSingleSupportState(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler,
                                              HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                                              WalkingControllerParameters walkingControllerParameters,
                                              WalkingFailureDetectionControlModule failureDetectionControlModule, double minimumTransferTime,
                                              YoVariableRegistry parentRegistry)
   {
      super(transferToSide, WalkingStateEnum.getWalkingTransferState(transferToSide), walkingMessageHandler, controllerToolbox, managerFactory,
            failureDetectionControlModule, parentRegistry);

      this.minimumTransferTime.set(minimumTransferTime);

      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      fractionOfTransferToCollapseLeg.set(walkingControllerParameters.getLegConfigurationParameters().getFractionOfTransferToCollapseLeg());
   }

   @Override
   public void doTransitionIntoAction()
   {
      adjustTimings();

      super.doTransitionIntoAction();

      boolean initialTransfer = isInitialTransfer();

      if (initialTransfer)
      {
         walkingMessageHandler.reportWalkingStarted();
         pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
      }
      // In middle of walking or leaving foot pose, pelvis is good leave it like that.
      else
         pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      for (int i = 0; i < 3; i++)
         balanceManager.addFootstepToPlan(walkingMessageHandler.peek(i), walkingMessageHandler.peekTiming(i));
      balanceManager.setICPPlanTransferToSide(transferToSide);
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      FootstepTiming footstepTiming = walkingMessageHandler.peekTiming(0);
      balanceManager.initializeICPPlanForTransfer(footstepTiming.getSwingTime(), footstepTiming.getTransferTime(), finalTransferTime);

      if (balanceManager.wasTimingAdjustedForReachability())
      {
         double currentTransferDuration = balanceManager.getCurrentTransferDurationAdjustedForReachability();
         double currentSwingDuration = balanceManager.getCurrentSwingDurationAdjustedForReachability();

         footstepTiming.setTimings(currentSwingDuration, currentTransferDuration);
      }

      pelvisOrientationManager.setUpcomingFootstep(walkingMessageHandler.peek(0));
      pelvisOrientationManager.initializeTransfer(transferToSide, footstepTiming.getTransferTime(), footstepTiming.getSwingTime());

      legConfigurationManager.beginStraightening(transferToSide);
      legConfigurationManager.setFullyExtendLeg(transferToSide, false);
   }

   @Override
   public void doAction()
   {
      super.doAction();

      FootstepTiming footstepTiming = walkingMessageHandler.peekTiming(0);
      if (footstepTiming != null)
      {
         double transferDuration = footstepTiming.getTransferTime();

         boolean pastMinimumTime = getTimeInCurrentState() > fractionOfTransferToCollapseLeg.getDoubleValue() * transferDuration;
         boolean isFootWellPosition = legConfigurationManager.areFeetWellPositionedForCollapse(transferToSide.getOppositeSide());
         if (pastMinimumTime && isFootWellPosition && !legConfigurationManager.isLegCollapsed(transferToSide.getOppositeSide()))
         {
            legConfigurationManager.collapseLegDuringTransfer(transferToSide);
         }
      }
   }

   @Override
   public boolean isDone()
   {
      return super.isDone() || feetManager.isFootToeingOffSlipping(transferToSide.getOppositeSide());
   }

   /**
    * This method checks if the upcoming step has a desired absolute start time. If that is the case
    * the transfer time is adjusted such that the swing starts at the correct time.
    */
   private void adjustTimings()
   {
      FootstepTiming stepTiming = walkingMessageHandler.peekTiming(0);
      double originalSwingTime = stepTiming.getSwingTime();

      if (!stepTiming.hasAbsoluteTime())
         return;

      double currentTime = controllerToolbox.getYoTime().getDoubleValue();
      double timeInFootstepPlan = currentTime - stepTiming.getExecutionStartTime();
      double adjustedTransferTime = stepTiming.getSwingStartTime() - timeInFootstepPlan;

      // make sure transfer does not get too short
      adjustedTransferTime = Math.max(adjustedTransferTime, minimumTransferTime.getDoubleValue());

      // GW TODO - possible improvement:
      // If the adjustment is capped by the minimum transfer time adjust also the upcoming transfer times here. That
      // would make the ICP plan for the upcoming steps more accurate. However, if the given original transfer times
      // are correctly set this might be a minimal improvement that makes step timing more complicated and difficult
      // to debug. If we have big adjustments a lot we should revisit this.

      // keep swing times and only adjust transfers for now
      stepTiming.setTimings(originalSwingTime, adjustedTransferTime);
   }
}