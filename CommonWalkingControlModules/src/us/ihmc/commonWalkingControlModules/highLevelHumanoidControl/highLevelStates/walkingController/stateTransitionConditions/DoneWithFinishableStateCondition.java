package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.robotics.stateMachines.FinishableState;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class DoneWithFinishableStateCondition implements StateTransitionCondition
{
   private final FinishableState<?> state;

   public DoneWithFinishableStateCondition(FinishableState<?> state)
   {
      this.state = state;
   }

   @Override
   public boolean checkCondition()
   {
      return state.isDone();
   }
}