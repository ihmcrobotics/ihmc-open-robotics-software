package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;

public class TransferToWalkingSingleSupportState extends TransferState
{
   public TransferToWalkingSingleSupportState(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler,
         HighLevelHumanoidControllerToolbox momentumBasedController, HighLevelControlManagerFactory managerFactory,
         WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(transferToSide, WalkingStateEnum.getWalkingTransferState(transferToSide), walkingMessageHandler, momentumBasedController, managerFactory,
            failureDetectionControlModule, parentRegistry);
   }

   @Override
   public void doTransitionIntoAction()
   {
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
         balanceManager.addFootstepToPlan(walkingMessageHandler.peek(i));
      balanceManager.setICPPlanTransferToSide(transferToSide);
      double defaultSwingTime = walkingMessageHandler.getDefaultSwingTime();
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      balanceManager.initializeICPPlanForTransfer(defaultSwingTime, defaultTransferTime, finalTransferTime);
   }
}