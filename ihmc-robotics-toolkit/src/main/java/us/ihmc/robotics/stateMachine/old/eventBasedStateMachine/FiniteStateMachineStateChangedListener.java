package us.ihmc.robotics.stateMachine.old.eventBasedStateMachine;

@Deprecated
public interface FiniteStateMachineStateChangedListener
{
   void stateHasChanged(Enum<?> oldState, Enum<?> newState);
}
