package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController.SingleSupportState;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

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
   public boolean checkCondition()
   {
      if (!singleSupportState.isDone())
         return false;

      return !walkingMessageHandler.hasUpcomingFootsteps();
   }
}