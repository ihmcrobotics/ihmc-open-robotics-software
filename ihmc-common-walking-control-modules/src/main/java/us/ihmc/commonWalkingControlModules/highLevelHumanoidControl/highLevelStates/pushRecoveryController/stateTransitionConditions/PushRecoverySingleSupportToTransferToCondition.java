package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.SingleSupportPushRecoveryState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.TransferToRecoveringSingleSupportState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class PushRecoverySingleSupportToTransferToCondition implements StateTransitionCondition
{
   private final SingleSupportPushRecoveryState singleSupportState;
   private final TransferToRecoveringSingleSupportState transferState;
   private final WalkingMessageHandler walkingMessageHandler;

   public PushRecoverySingleSupportToTransferToCondition(SingleSupportPushRecoveryState singleSupportState, TransferToRecoveringSingleSupportState transferState, WalkingMessageHandler walkingMessageHandler)
   {
      this.singleSupportState = singleSupportState;
      this.transferState = transferState;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!singleSupportState.isDone(timeInState))
         return false;
      return walkingMessageHandler.isNextFootstepFor(transferState.getTransferToSide().getOppositeSide());
   }
}