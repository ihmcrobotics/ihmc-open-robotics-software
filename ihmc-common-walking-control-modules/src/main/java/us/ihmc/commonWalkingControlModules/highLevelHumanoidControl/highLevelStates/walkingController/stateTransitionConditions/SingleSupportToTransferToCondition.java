package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.SingleSupportState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferState;

public class SingleSupportToTransferToCondition implements StateTransitionCondition
{
   private final SingleSupportState singleSupportState;
   private final TransferState transferState;
   private final WalkingMessageHandler walkingMessageHandler;

   public SingleSupportToTransferToCondition(SingleSupportState singleSupportState, TransferState transferState, WalkingMessageHandler walkingMessageHandler)
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