package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;

@FunctionalInterface
public interface StateTransitionToAny<K>
{
   K shouldTransitionTo(double timeInCurrentState);
}
