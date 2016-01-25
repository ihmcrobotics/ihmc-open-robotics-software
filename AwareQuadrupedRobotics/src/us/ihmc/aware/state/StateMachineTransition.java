package us.ihmc.aware.state;

/**
 * Defines a state machine edge, specifying a source state and event, along with the destination state.
 *
 * @param <S> the state type.
 * @param <E> the event enum type.
 */
public class StateMachineTransition<S, E extends Enum<E>>
{
   /**
    * The event on which to transition.
    */
   private final E event;

   /**
    * The source state from which to transition.
    */
   private final S from;

   /**
    * The destination state to which to transition.
    */
   private final S to;

   public StateMachineTransition(E event, S from, S to)
   {
      this.event = event;
      this.from = from;
      this.to = to;
   }

   /**
    * {@see #event}
    */
   public E getEvent()
   {
      return event;
   }

   /**
    * {@see #from}
    */
   public S getFrom()
   {
      return from;
   }

   /**
    * {@see #to}
    */
   public S getTo()
   {
      return to;
   }
}
