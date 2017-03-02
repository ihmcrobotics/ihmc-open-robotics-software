package us.ihmc.robotics.stateMachines.eventBasedStateMachine;

/**
 * Defines a callback for a given state as triggered by an event. Note a callback does not necessarily result in a transition.
 *
 * @param <E> the event enum type.
 */
public class FiniteStateMachineCallback<S extends Enum<S>, E extends Enum<E>>
{
   /**
    * The event on which to trigger.
    */
   private final E event;

   /**
    * The state on which to trigger.
    */
   private final S state;

   /**
    * The callback function.
    */
   private final Runnable callback;

   public FiniteStateMachineCallback(E event, S state, Runnable callback)
   {
      this.event = event;
      this.state = state;
      this.callback = callback;
   }

   /**
    * {@see #event}
    */
   public E getEvent()
   {
      return event;
   }

   /**
    * {@see #state}
    */
   public S getState()
   {
      return state;
   }

   /**
    * {@see #function}
    */
   public void call()
   {
      this.callback.run();
   }
}
