package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveringSwingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveryTransferState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class PushRecoverySingleSupportToTransferToCondition implements StateTransitionCondition
{
   private final RecoveringSwingState singleSupportState;
   private final RecoveryTransferState transferState;
   private final WalkingMessageHandler walkingMessageHandler;

   public PushRecoverySingleSupportToTransferToCondition(RecoveringSwingState singleSupportState, RecoveryTransferState transferState, WalkingMessageHandler walkingMessageHandler)
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