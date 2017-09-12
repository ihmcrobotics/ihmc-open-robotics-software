package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlamingoStanceState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class StopFlamingoCondition implements StateTransitionCondition
{
   private final FlamingoStanceState singleSupportState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopFlamingoCondition(FlamingoStanceState singleSupportState, WalkingMessageHandler walkingMessageHandler)
   {
      this.singleSupportState = singleSupportState;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean checkCondition()
   {
      if (!singleSupportState.isDone())
         return false;

      return !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(singleSupportState.getSwingSide());
   }
}