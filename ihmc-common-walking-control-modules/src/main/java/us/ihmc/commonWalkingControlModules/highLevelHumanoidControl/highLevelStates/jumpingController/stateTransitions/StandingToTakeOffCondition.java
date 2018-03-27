package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionAction;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class StandingToTakeOffCondition implements StateTransitionCondition, StateTransitionAction
{
   boolean standingForTheFirstTime = true;
   
   @Override
   public boolean checkCondition()
   {
      return false; //!standingForTheFirstTime;
   }

   @Override
   public void doTransitionAction()
   {
      standingForTheFirstTime = false;
   }
}
