package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.stateMachine.core.StateTransitionCondition;

public class StartWalkingCondition implements StateTransitionCondition
{
   private final RobotSide transferToSide;
   private final WalkingMessageHandler walkingMessageHandler;

   public StartWalkingCondition(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler)
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