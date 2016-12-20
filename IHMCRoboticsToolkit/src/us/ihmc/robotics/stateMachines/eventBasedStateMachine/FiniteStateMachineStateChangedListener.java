package us.ihmc.robotics.stateMachines.eventBasedStateMachine;

public interface FiniteStateMachineStateChangedListener
{
   void stateHasChanged(Enum<?> oldState, Enum<?> newState);
}
