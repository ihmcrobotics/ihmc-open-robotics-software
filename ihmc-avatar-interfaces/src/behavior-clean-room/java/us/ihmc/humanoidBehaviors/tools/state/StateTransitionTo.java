package us.ihmc.humanoidBehaviors.tools.state;

/**
 * State transition condition to switch to any state by writing only one function.
 */
@FunctionalInterface
public interface StateTransitionTo<K>
{
   K shouldTransitionTo(double timeInCurrentState);
}
