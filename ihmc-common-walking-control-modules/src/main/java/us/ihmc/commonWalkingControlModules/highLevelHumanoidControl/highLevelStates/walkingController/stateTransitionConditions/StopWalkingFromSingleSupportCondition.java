package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.SingleSupportState;

public class StopWalkingFromSingleSupportCondition implements StateTransitionCondition
{
   private final SingleSupportState singleSupportState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopWalkingFromSingleSupportCondition(SingleSupportState singleSupportState, WalkingMessageHandler walkingMessageHandler)
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