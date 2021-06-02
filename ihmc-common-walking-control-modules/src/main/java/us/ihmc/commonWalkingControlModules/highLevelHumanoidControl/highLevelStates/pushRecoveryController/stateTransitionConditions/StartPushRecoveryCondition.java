package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StartPushRecoveryCondition implements StateTransitionCondition
{
   private final RobotSide transferToSide;
   private final WalkingMessageHandler walkingMessageHandler;

   public StartPushRecoveryCondition(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler)
   {
      this.transferToSide = transferToSide;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      return walkingMessageHandler.isNextFootstepFor(transferToSide.getOppositeSide());
   }
}