package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

public class DoneWithFinishableStateTransitionCondition implements StateTransitionCondition
{
   private final FinishableState<?> state;

   public DoneWithFinishableStateTransitionCondition(FinishableState<?> state)
   {
      this.state = state;
   }

   @Override
   public boolean checkCondition()
   {
      return state.isDone();
   }
}
