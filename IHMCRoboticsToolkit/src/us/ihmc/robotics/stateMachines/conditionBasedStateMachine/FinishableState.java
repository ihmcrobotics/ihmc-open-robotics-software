package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

public abstract class FinishableState<E extends Enum<E>> extends State<E>
{
   public FinishableState(E stateEnum)
   {
      super(stateEnum);
   }

   public abstract boolean isDone();

   public final void addDoneWithStateTransition(final E nextStateEnum)
   {
      StateTransitionCondition stateTransitionCondition = new DoneWithFinishableStateTransitionCondition(this);
      this.addStateTransition(nextStateEnum, stateTransitionCondition);
   }
}
