package us.ihmc.robotics.stateMachine.old.eventBasedStateMachine;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * A finite state machine implementation capable of holding states of type {@link S} and firing events of any arbitrary enum type.
 *
 * @param <S> the state enum type
 * @param <E> the default event enum type, for convenience
 */
@Deprecated
public class FiniteStateMachine<S extends Enum<S>, E extends Enum<E>, C extends FiniteStateMachineState<E>>
{
   private final Map<S, C> states;

   /**
    * The list of possible transitions. This is equivalent to a state-transition function in FSM literature.
    */
   /*
    * NOTE: This should be a {@link java.util.Set}, but due to real-time constraints a {@link List} must be used
    * instead.
    */
   private final Map<Class<?>, List<FiniteStateMachineTransition<S, ? extends Enum<?>>>> transitions;

   /**
    * The list of optional callback functions to handle events triggered in a specific state.
    */
   /*
    * NOTE: This should be a {@link java.util.Set}, but due to real-time constraints a {@link List} must be used
    * instead.
    */
   private final Map<Class<?>, List<FiniteStateMachineCallback<S, ? extends Enum<?>>>> callbacks;

   /**
    * The list of state changed listeners to be notified of each state transition.
    */
   private final ArrayList<FiniteStateMachineStateChangedListener> stateChangedListeners;

   /**
    * The default type of events, {@link E}. Specifying this is a convenience that allows the type to be omitted from calls to {@link #trigger(Enum)} unless the
    * event is of a different type.
    */
   private final Class<E> standardEventType;

   /**
    * The state to be called on the first call to {@link #process()}, assuming it has not been overridden.
    */
   private final S initialState;

   /**
    * The present state.
    */
   private YoEnum<S> stateEnum;

   /**
    * Whether or not the current state's {@link FiniteStateMachineState#onEntry()} needs to be called at the beginning of the next {@link #process()} call. This
    * is required because we don't want to call it immediately when the transition occurs. Rather, we want to wait until the next control cycle so the state's
    * {@link FiniteStateMachineState#onEntry()} and {@link FiniteStateMachineState#process()} methods are called in the same control loop.
    */
   // True so we don't forget to initialize the first state at startup
   private boolean needToCallOnEntry = true;

   /**
    * Use {@link FiniteStateMachineBuilder} instead.
    */
   FiniteStateMachine(Map<S, C> states, Map<Class<?>, List<FiniteStateMachineTransition<S, ? extends Enum<?>>>> transitions,
         Map<Class<?>, List<FiniteStateMachineCallback<S, ? extends Enum<?>>>> callbacks, S initialState, Class<S> enumType, Class<E> standardEventType,
         String yoVariableName, YoVariableRegistry registry)
   {
      this.states = states;
      this.transitions = transitions;
      this.callbacks = callbacks;
      this.stateChangedListeners = new ArrayList<>();
      this.initialState = initialState;
      this.standardEventType = standardEventType;
      this.stateEnum = new YoEnum<>(yoVariableName, registry, enumType);
      this.stateEnum.set(initialState);
   }

   /**
    * Add a state change listener to be notified of each state transition.
    * @param listener the state change listener
    */
   public void attachStateChangedListener(FiniteStateMachineStateChangedListener listener)
   {
      stateChangedListeners.add(listener);
   }

   /**
    * Trigger the given event and follow the state transition, if it exists.
    * <p/>
    * If no transition is defined for the present state and given event then no action will be taken.
    *
    * @param event the triggered event
    */
   public void trigger(E event)
   {
      trigger(standardEventType, event);
   }

   /**
    * Trigger an event of an arbitrary type.
    *
    * @param type  the class of {@link M}
    * @param event the triggered event
    * @param <M>   the type of the event
    * @see #trigger(Enum)
    */
   public <M extends Enum<M>> void trigger(Class<M> type, M event)
   {
      List<FiniteStateMachineCallback<S, ? extends Enum<?>>> callbacksTypeM = callbacks.get(type);
      if (callbacksTypeM != null)
      {
         for (int i = 0; i < callbacksTypeM.size(); i++)
         {
            FiniteStateMachineCallback<S, ?> callback = callbacksTypeM.get(i);

            // Check if this callback should be called.
            if (callback.getState() == getCurrentStateEnum() && event == callback.getEvent())
            {
               callback.call();
            }
         }
      }

      List<FiniteStateMachineTransition<S, ? extends Enum<?>>> transitionsTypeM = transitions.get(type);
      if (transitionsTypeM != null)
      {
         for (int i = 0; i < transitionsTypeM.size(); i++)
         {
            FiniteStateMachineTransition<S, ?> transition = transitionsTypeM.get(i);

            // Check if this transition matches the source state and event.
            if (transition.getFrom() == getCurrentStateEnum() && event == transition.getEvent())
            {
               for (int j = 0; j < stateChangedListeners.size(); j++)
               {
                  stateChangedListeners.get(j).stateHasChanged(transition.getFrom(), transition.getTo());
               }
               transition(transition.getFrom(), transition.getTo());
               break;
            }
         }
      }
   }

   /**
    * Run the current state's {@link FiniteStateMachineState#process()} method and transition on any generated events.
    */
   public void process()
   {
      C instance = states.get(getCurrentStateEnum());

      // Call the delayed onEntry() function at the beginning of the process(), rather than at the end of the previous process().
      if (needToCallOnEntry)
      {
         instance.onEntry();
         needToCallOnEntry = false;
      }

      // Run the current state and see if it generates an event.
      E event = instance.process();

      if (event != null)
      {
         trigger(standardEventType, event);
      }
   }

   /**
    * {@see #state}
    */
   public S getCurrentStateEnum()
   {
      return stateEnum.getEnumValue();
   }

   /**
    * Forcefully set the current state.
    * <p/>
    * NOTE: Use this method with caution. It does not enforce reachability of the new state.
    *
    * @param stateEnum the new state
    */
   public void setState(S stateEnum)
   {
      this.stateEnum.set(stateEnum);
   }

   /**
    * Resets the state machine to the initial state, regardless of whether or not there is a transition to follow. Still calls {@link
    * FiniteStateMachineState#onEntry()} and {@link FiniteStateMachineState#onExit()} to ensure states are cleaned up.
    */
   public void reset()
   {
      transition(getCurrentStateEnum(), initialState);
   }

   private FiniteStateMachineState<?> getInstanceForEnum(S state)
   {
      if (!states.containsKey(state))
      {
         throw new IllegalArgumentException("State " + state + " is not registered");
      }

      return states.get(state);
   }

   private void transition(S from, S to)
   {
      FiniteStateMachineState<?> fromInstance = getInstanceForEnum(from);

      fromInstance.onExit();
      setState(to);

      // Delay call to onEntry until beginning of next process().
      needToCallOnEntry = true;
   }
   
   public C getState(S stateEnum)
   {
      return states.get(stateEnum);
   }

   public C getCurrentState()
   {
      return states.get(getCurrentStateEnum());
   }
}
