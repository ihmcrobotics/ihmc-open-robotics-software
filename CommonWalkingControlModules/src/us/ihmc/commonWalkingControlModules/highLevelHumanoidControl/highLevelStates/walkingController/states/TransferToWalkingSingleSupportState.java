package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class TransferToWalkingSingleSupportState extends TransferState
{
   private final DoubleYoVariable minimumTransferTime = new DoubleYoVariable("minimumTransferTime", registry);

   public TransferToWalkingSingleSupportState(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler,
         HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
         WalkingFailureDetectionControlModule failureDetectionControlModule, double minimumTransferTime, YoVariableRegistry parentRegistry)
   {
      super(transferToSide, WalkingStateEnum.getWalkingTransferState(transferToSide), walkingMessageHandler, controllerToolbox, managerFactory,
            failureDetectionControlModule, parentRegistry);

      this.minimumTransferTime.set(minimumTransferTime);
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
      double defaultSwingTime = walkingMessageHandler.getDefaultSwingTime();
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      balanceManager.initializeICPPlanForTransfer(defaultSwingTime, defaultTransferTime, finalTransferTime);
   }

   /**
    * This method checks if the upcoming step has a desired absolute start time. If that is the case the transfer time is
    * adjusted such that the swing starts at the correct time.
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