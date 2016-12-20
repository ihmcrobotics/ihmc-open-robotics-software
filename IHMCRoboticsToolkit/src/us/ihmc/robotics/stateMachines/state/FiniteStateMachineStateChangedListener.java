package us.ihmc.robotics.stateMachines.state;

public interface FiniteStateMachineStateChangedListener
{
   void stateHasChanged(Enum<?> oldState, Enum<?> newState);
}
