package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveringSwingState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StopPushRecoveryFromSingleSupportCondition implements StateTransitionCondition
{
   private final RecoveringSwingState singleSupportState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopPushRecoveryFromSingleSupportCondition(RecoveringSwingState singleSupportState, WalkingMessageHandler walkingMessageHandler)
   {
      this.singleSupportState = singleSupportState;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!singleSupportState.isDone(timeInState))
         return false;

      return !walkingMessageHandler.hasUpcomingFootsteps();
   }
}