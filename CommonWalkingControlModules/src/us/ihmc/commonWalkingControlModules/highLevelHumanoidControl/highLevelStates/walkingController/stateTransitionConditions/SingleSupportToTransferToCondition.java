package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.SingleSupportState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

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
   public boolean checkCondition()
   {
      if (!singleSupportState.isDone())
         return false;
      return walkingMessageHandler.isNextFootstepFor(transferState.getTransferToSide().getOppositeSide());
   }
}