package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;

@Deprecated
public interface StateChangedListener <E extends Enum<E>>
{
   public abstract void stateChanged(State<E> oldState, State<E> newState, double time);
}
