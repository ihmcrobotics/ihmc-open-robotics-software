package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;

@FunctionalInterface
public interface StateTransitionTo<K>
{
   K shouldTransitionTo(double timeInCurrentState);
}
