package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TransferState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.stateMachine.core.StateTransitionCondition;

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
   public boolean testCondition(double timeInState)
   {
      RobotSide trailingFootSide = transferState.getTransferToSide().getOppositeSide();
      boolean noFootstep = !walkingMessageHandler.isNextFootstepFor(trailingFootSide);

      return noFootstep;
   }
}