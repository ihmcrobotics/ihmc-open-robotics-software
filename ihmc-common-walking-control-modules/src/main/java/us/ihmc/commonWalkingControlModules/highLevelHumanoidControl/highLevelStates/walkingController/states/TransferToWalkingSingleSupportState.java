package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
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
   }

   @Override
   protected void updateICPPlan()
   {
      super.updateICPPlan();
      boolean initialTransfer = isInitialTransfer();

      if (initialTransfer)
      {
         walkingMessageHandler.reportWalkingStarted();
         pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
      }
      // In middle of walking or leaving foot pose, pelvis is good leave it like that.
      else
         pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

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
            walkingMessageHandler.adjustTiming(timing.getSwingTime(), timing.getTouchdownDuration(), timing.getTransferTime());
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
         double currentTouchdownDuration = balanceManager.getCurrentTouchdownDuration();

         firstTiming.setTimings(currentSwingDuration, currentTouchdownDuration, currentTransferDuration);
      }

      pelvisOrientationManager.setUpcomingFootstep(footsteps[0]);
      pelvisOrientationManager.initializeTransfer(transferToSide, firstTiming.getTransferTime(), firstTiming.getSwingTime());

      legConfigurationManager.beginStraightening(transferToSide);
      legConfigurationManager.setFullyExtendLeg(transferToSide, false);
   }

   @Override
   public void doAction(double timeInState)
   {
      super.doAction(timeInState);

      double transferDuration = currentTransferDuration.getDoubleValue();
      boolean pastMinimumTime = timeInState > fractionOfTransferToCollapseLeg.getDoubleValue() * transferDuration;
      boolean isFootWellPosition = legConfigurationManager.areFeetWellPositionedForCollapse(transferToSide.getOppositeSide());
      if (pastMinimumTime && isFootWellPosition && !legConfigurationManager.isLegCollapsed(transferToSide.getOppositeSide()))
      {
         legConfigurationManager.collapseLegDuringTransfer(transferToSide);
      }
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return super.isDone(timeInState) || feetManager.isFootToeingOffSlipping(transferToSide.getOppositeSide());
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
      double originalTouchdownDuration = stepTiming.getTouchdownDuration();
      this.originalTransferTime.set(originalTransferTime);

      double currentTime = controllerToolbox.getYoTime().getDoubleValue();
      double timeInFootstepPlan = currentTime - stepTiming.getExecutionStartTime();
      double adjustedTransferTime = stepTiming.getSwingStartTime() - timeInFootstepPlan;

      // make sure transfer does not get too short
      adjustedTransferTime = Math.max(adjustedTransferTime, minimumTransferTime.getValue());

      // as the touchdown in part of transfer scale it according to the transfer adjustment
      double adjustmentFactor = adjustedTransferTime / originalTransferTime;
      double adjustedTouchdownDuration = originalTouchdownDuration * adjustmentFactor;

      // GW TODO - possible improvement:
      // If the adjustment is capped by the minimum transfer time adjust also the upcoming transfer times here. That
      // would make the ICP plan for the upcoming steps more accurate. However, if the given original transfer times
      // are correctly set this might be a minimal improvement that makes step timing more complicated and difficult
      // to debug. If we have big adjustments a lot we should revisit this.

      // keep swing times and only adjust transfers for now
      stepTiming.setTimings(originalSwingTime, adjustedTouchdownDuration, adjustedTransferTime);
   }
}