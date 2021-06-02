package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.SingleSupportPushRecoveryState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StopPushRecoveryFromSingleSupportCondition implements StateTransitionCondition
{
   private final SingleSupportPushRecoveryState singleSupportState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopPushRecoveryFromSingleSupportCondition(SingleSupportPushRecoveryState singleSupportState, WalkingMessageHandler walkingMessageHandler)
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