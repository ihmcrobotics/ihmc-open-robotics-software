package us.ihmc.robotics.stateMachine.extra;

import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;

/**
 * Trigger used to fire events which in turn will potentially trigger state transitions for a state
 * machine.
 * <p>
 * Use the {@link EventBasedStateMachineFactory} to configure and create an event-based state
 * machine, and to to create {@link EventTrigger}s.
 * </p>
 * 
 * @author Sylvain
 *
 */
public interface EventTrigger
{
   /**
    * Call this method to fire events at a state machine to trigger its state transitions.
    * 
    * @param event the even to be fired.
    */
   <E extends Enum<E>> void fireEvent(E event);
}
