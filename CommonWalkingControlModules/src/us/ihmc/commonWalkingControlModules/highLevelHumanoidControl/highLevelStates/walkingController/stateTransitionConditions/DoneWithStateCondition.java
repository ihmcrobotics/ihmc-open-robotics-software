package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class DoneWithStateCondition implements StateTransitionCondition
{
   private final State<?> state;

   public DoneWithStateCondition(State<?> state)
   {
      this.state = state;
   }

   @Override
   public boolean checkCondition()
   {
      return state.isDone();
   }
}