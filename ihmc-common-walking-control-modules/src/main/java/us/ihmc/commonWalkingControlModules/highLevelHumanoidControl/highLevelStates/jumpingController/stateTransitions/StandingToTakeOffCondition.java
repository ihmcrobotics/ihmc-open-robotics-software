package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.StandingState;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionAction;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;

public class StandingToTakeOffCondition implements StateTransitionCondition, StateTransitionAction
{
   private final StandingState standingState;
   
   public StandingToTakeOffCondition(StandingState standingState)
   {
      this.standingState = standingState;
   }
   
   @Override
   public boolean checkCondition()
   {
      return standingState.isDone();
   }

   @Override
   public void doTransitionAction()
   {
      
   }
}
