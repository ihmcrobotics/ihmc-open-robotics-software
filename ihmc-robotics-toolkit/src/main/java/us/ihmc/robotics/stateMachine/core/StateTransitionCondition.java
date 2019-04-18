package us.ihmc.robotics.stateMachine.core;

/**
 * Interface used for implementing state transition conditions.
 * <p>
 * State transition conditions are registered in {@link StateTransition}s which are then registered
 * to a {@link StateMachine}.
 * </p>
 * <p>
 * When registering a state transition condition, it is associated with the keys of a source state
 * and target state. When the source state is active and the condition is fulfilled, i.e.
 * {@code #testCondition(double)} returns {@code true}, the state machine will trigger a transition
 * to the target state.
 * </p>
 * 
 * @author Sylvain
 */
@FunctionalInterface
public interface StateTransitionCondition
{
   /**
    * Invoked by the state machine to identify if a transition to the target state associated with this
    * condition is requested.
    * 
    * @param timeInCurrentState the time spent in the current state, or {@link Double#NaN} if the time
    *           information is unavailable.
    * @return {@code true} to request a transition, {@code false} otherwise.
    */
   boolean testCondition(double timeInCurrentState);
}
