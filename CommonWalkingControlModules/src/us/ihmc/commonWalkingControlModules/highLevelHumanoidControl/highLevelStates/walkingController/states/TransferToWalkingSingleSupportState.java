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
         HighLevelHumanoidControllerToolbox momentumBasedController, HighLevelControlManagerFactory managerFactory,
         WalkingFailureDetectionControlModule failureDetectionControlModule, double minimumTransferTime, YoVariableRegistry parentRegistry)
   {
      super(transferToSide, WalkingStateEnum.getWalkingTransferState(transferToSide), walkingMessageHandler, momentumBasedController, managerFactory,
            failureDetectionControlModule, parentRegistry);

      this.minimumTransferTime.set(minimumTransferTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      // to ensure that the swing starts at the desired absolute time this will adjust the transfer time
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

      double currentTime = momentumBasedController.getYoTime().getDoubleValue();
      double timeInFootstepPlan = currentTime - stepTiming.getExecutionStartTime();
      double adjustedTransferTime = stepTiming.getSwingStartTime() - timeInFootstepPlan;

      // make sure transfer does not get too short
      adjustedTransferTime = Math.max(adjustedTransferTime, minimumTransferTime.getDoubleValue());

      // keep swing times and only adjust transfers for now
      stepTiming.setTimings(originalSwingTime, adjustedTransferTime);
   }
}