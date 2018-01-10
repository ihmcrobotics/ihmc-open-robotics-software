package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferToWalkingSingleSupportState extends TransferState
{
   private static final int numberOfFootstepsToConsider = 3;
   private final Footstep[] footsteps = Footstep.createFootsteps(numberOfFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(numberOfFootstepsToConsider);

   private final YoDouble minimumTransferTime = new YoDouble("minimumTransferTime", registry);

   private final LegConfigurationManager legConfigurationManager;
   private final YoDouble fractionOfTransferToCollapseLeg = new YoDouble("fractionOfTransferToCollapseLeg", registry);
   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   public TransferToWalkingSingleSupportState(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler,
                                              HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                                              WalkingControllerParameters walkingControllerParameters,
                                              WalkingFailureDetectionControlModule failureDetectionControlModule, double minimumTransferTime,
                                              YoVariableRegistry parentRegistry)
   {
      super(transferToSide, WalkingStateEnum.getWalkingTransferState(transferToSide), walkingControllerParameters, walkingMessageHandler, controllerToolbox, managerFactory,
            failureDetectionControlModule, parentRegistry);

      this.minimumTransferTime.set(minimumTransferTime);

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

      walkingMessageHandler.requestPlanarRegions();

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
            walkingMessageHandler.adjustTimings(0, timing.getSwingTime(), timing.getTouchdownDuration(), timing.getTransferTime());
         }

         balanceManager.addFootstepToPlan(footstep, timing);
      }

      balanceManager.setICPPlanTransferToSide(transferToSide);
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
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
   public void doAction()
   {
      super.doAction();

      double transferDuration = currentTransferDuration.getDoubleValue();
      boolean pastMinimumTime = getTimeInCurrentState() > fractionOfTransferToCollapseLeg.getDoubleValue() * transferDuration;
      boolean isFootWellPosition = legConfigurationManager.areFeetWellPositionedForCollapse(transferToSide.getOppositeSide());
      if (pastMinimumTime && isFootWellPosition && !legConfigurationManager.isLegCollapsed(transferToSide.getOppositeSide()))
      {
         legConfigurationManager.collapseLegDuringTransfer(transferToSide);
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
   private void adjustTiming(FootstepTiming stepTiming)
   {
      double originalSwingTime = stepTiming.getSwingTime();

      if (!stepTiming.hasAbsoluteTime())
         return;

      double currentTime = controllerToolbox.getYoTime().getDoubleValue();
      double timeInFootstepPlan = currentTime - stepTiming.getExecutionStartTime();
      double adjustedTransferTime = stepTiming.getSwingStartTime() - timeInFootstepPlan;

      // make sure transfer does not get too short
      adjustedTransferTime = Math.max(adjustedTransferTime, minimumTransferTime.getDoubleValue());
      double touchdownDuration = stepTiming.getTouchdownDuration();

      // GW TODO - possible improvement:
      // If the adjustment is capped by the minimum transfer time adjust also the upcoming transfer times here. That
      // would make the ICP plan for the upcoming steps more accurate. However, if the given original transfer times
      // are correctly set this might be a minimal improvement that makes step timing more complicated and difficult
      // to debug. If we have big adjustments a lot we should revisit this.

      // keep swing times and only adjust transfers for now
      stepTiming.setTimings(originalSwingTime, touchdownDuration, adjustedTransferTime);
   }
}