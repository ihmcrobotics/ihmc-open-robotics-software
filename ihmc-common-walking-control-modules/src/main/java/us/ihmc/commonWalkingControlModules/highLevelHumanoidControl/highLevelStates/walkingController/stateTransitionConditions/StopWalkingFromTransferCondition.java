package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class StopWalkingFromTransferCondition implements StateTransitionCondition
{
   private final TransferState transferState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopWalkingFromTransferCondition(TransferState transferState, WalkingMessageHandler walkingMessageHandler)
   {
      this.transferState = transferState;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean checkCondition()
   {
      RobotSide trailingFootSide = transferState.getTransferToSide().getOppositeSide();
      boolean noFootstep = !walkingMessageHandler.isNextFootstepFor(trailingFootSide);
      
      return noFootstep && transferState.isStopWalkingSafe();
   }
}