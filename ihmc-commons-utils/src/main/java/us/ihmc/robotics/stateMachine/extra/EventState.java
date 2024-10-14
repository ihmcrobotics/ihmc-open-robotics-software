package us.ihmc.robotics.stateMachine.extra;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;

/**
 * As a {@link State}, an {@link EventState} is used to create a {@link StateMachine}.
 * <p>
 * When used with a {@link EventBasedStateMachineFactory} to build a new state machine,
 * {@code EventState} provides an alternative to the usual state transitions. Instead of
 * implementing state transition conditions, the user registers a series of events which will
 * trigger specific transitions from a given source state to a given target state.
 * {@code EventState} adds to possibility to fire these events from within the active state.
 * </p>
 * 
 * @author Sylvain
 */
public interface EventState extends State
{
   /**
    * Invoked frequently by a state machine to collect events to be processed.
    * <p>
    * When an event is fired, the state machine will then look for a transition associated with it. If
    * there exists a transition with a target state, the state machine will then perform a state change
    * to that target state. Otherwise, this state will remains the active state.
    * </p>
    * 
    * @param timeInState the time spent in this state or {@link Double#NaN} if the time information is
    *           unavailable.
    * @return the event to be processed by the state machine, or {@code null} if no event is to be
    *         fired.
    */
   Enum<?> fireEvent(double timeInState);
}
