package us.ihmc.aware.state;

import java.util.List;

public class StateMachine<S, E extends Enum<E>>
{
   /**
    * The list of possible transitions. This is equivalent to a state-transition function in FSM literature.
    */
   /*
    * NOTE: This should be a {@link java.util.Set}, but due to real-time constraints a {@link List} must be used
    * instead.
    */
   private final List<StateMachineTransition<S, E>> transitions;

   /**
    * The present state.
    */
   private S state;

   public StateMachine(List<StateMachineTransition<S, E>> transitions, S initialState)
   {
      this.transitions = transitions;
      this.state = initialState;
   }

   /**
    * Trigger the given event and follow the state transition, if it exists.
    * <p/>
    * If no transition is defined for the present state and given event then no action will be taken.
    *
    * @param event the triggered event.
    */
   public void trigger(E event)
   {
      for (int i = 0; i < transitions.size(); i++)
      {
         StateMachineTransition<S, E> transition = transitions.get(i);

         // Check if this transition matches the source state and event.
         if (transition.getFrom().equals(state) && event.equals(transition.getEvent()))
         {
            // It does, so transition to the next state.
            state = transition.getTo();
         }
      }
   }

   /**
    * {@see #state}
    */
   public S getState()
   {
      return state;
   }
}
