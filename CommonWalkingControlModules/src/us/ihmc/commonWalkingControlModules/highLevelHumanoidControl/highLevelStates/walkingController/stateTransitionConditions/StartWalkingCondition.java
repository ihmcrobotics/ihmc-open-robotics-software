package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

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
   public boolean checkCondition()
   {
      boolean transferringToThisRobotSide;
      if (walkingMessageHandler.hasUpcomingFootsteps())
         transferringToThisRobotSide = transferToSide == walkingMessageHandler.peek(0).getRobotSide().getOppositeSide();
      else
         transferringToThisRobotSide = false;

      return transferringToThisRobotSide;
   }
}