package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.PushRecoveryFlamingoStanceState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlamingoStanceState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StopFlamingoRecoveryCondition implements StateTransitionCondition
{
   private final PushRecoveryFlamingoStanceState singleSupportState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopFlamingoRecoveryCondition(PushRecoveryFlamingoStanceState singleSupportState, WalkingMessageHandler walkingMessageHandler)
   {
      this.singleSupportState = singleSupportState;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!singleSupportState.isDone(timeInState))
         return false;

      return !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(singleSupportState.getSwingSide());
   }
}