package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlamingoStanceState;

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
   public boolean testCondition(double timeInState)
   {
      if (!singleSupportState.isDone(timeInState))
         return false;

      return !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(singleSupportState.getSwingSide())
             && !walkingMessageHandler.hasLegTrajectoryForFlamingoStance(singleSupportState.getSwingSide());
   }
}