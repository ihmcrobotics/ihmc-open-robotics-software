package us.ihmc.aware.state;

import java.util.List;
import java.util.Map;

public class StateMachine<S extends Enum<S>, E extends Enum<E>>
{
   private final Map<S, StateMachineState<E>> states;

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

   /**
    * Whether or not the initial state's {@link StateMachineState#onEntry()} method has been called.
    */
   private boolean initialized = false;

   public StateMachine(Map<S, StateMachineState<E>> states, List<StateMachineTransition<S, E>> transitions,
         S initialState)
   {
      this.states = states;
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
         if (transition.getFrom() == state && event == transition.getEvent())
         {
            StateMachineState<E> fromInstance = getInstanceForEnum(transition.getFrom());
            StateMachineState<E> toInstance = getInstanceForEnum(transition.getTo());

            // It does, so transition to the next state.
            fromInstance.onExit();
            state = transition.getTo();
            toInstance.onEntry();
         }
      }
   }

   /**
    * Run the current state's {@link StateMachineState#process()} method and transition on any generated events.
    */
   public void process()
   {
      // Call the initial state's onEntry if it has not been called yet.
      if(!initialized)
      {
         StateMachineState<E> instance = getInstanceForEnum(state);
         instance.onEntry();
         initialized = true;
      }

      StateMachineState<E> instance = states.get(state);

      // Run the current state and see if it generates an event.
      E event = instance.process();
      if (event != null)
      {
         trigger(event);
      }
   }

   /**
    * {@see #state}
    */
   public S getState()
   {
      return state;
   }

   private StateMachineState<E> getInstanceForEnum(S state)
   {
      if (!states.containsKey(state))
      {
         throw new IllegalArgumentException("State " + state + " is not registered");
      }

      return states.get(state);
   }
}
