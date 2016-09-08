package us.ihmc.quadrupedRobotics.state;

public interface FiniteStateMachineStateChangedListener
{
   void stateHasChanged(Enum<?> oldState, Enum<?> newState);
}
