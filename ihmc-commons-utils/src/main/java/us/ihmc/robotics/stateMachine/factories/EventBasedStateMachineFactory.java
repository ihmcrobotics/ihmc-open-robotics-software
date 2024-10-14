package us.ihmc.robotics.stateMachine.factories;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.extra.EventState;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This factory gathers convenience methods to configure and create a {@link StateMachine}.
 * <p>
 * Unlike {@link StateMachineFactory}, this factory configures the state machine to work with an
 * event-based framework. Instead of configuring transitions with user-defined conditions, the user
 * can define the response of the state machine when a specific event is fired.
 * </p>
 * <p>
 * Events can be pretty much any object. However, it is recommended to use enums which provide a
 * human readable type. For instance, a good practice is to use enum like {@code MyStateMachineEvent
 * {ALL_GOOD, FAILED}} where the name of the enum clearly indicates it is used for events only and
 * the constants indicates some status.
 * </p>
 * <p>
 * Aside the fact that transitions are triggered via events, the state machine works the way as a
 * state machine created using {@link StateMachineFactory}. The user will have to call
 * {@link StateMachine#doActionAndTransition()} on a regular basis to run the active state and
 * perform transitions.
 * </p>
 * <p>
 * Here is an example for creating a simple state machine that represents the main usecase of this
 * factory:
 * <ol>
 * <li>Create new factory: <br>
 * {@code EventBasedStateMachineFactory<StateEnum, State> factory = new EventBasedStateMachineFactory<>(StateEnum.class);}
 * <li>Setup the name, registry, and clock:<br>
 * {@code factory.setNamePrefix("myStateMachine").setRegistry(myRegistry).buildYoClock(yoTime);}
 * <li>Add the states:<br>
 * {@code factory.addState(StateEnum.A, stateA).addState(StateEnum.B, stateB);}
 * <li>Configure the event responses:<br>
 * {@code factor.addTransition(MyEvent.DONE, StateEnum.A, StateEnum.B);}
 * <li>Create a trigger to fire events at the state machine from anywhere:<br>
 * {@code EventTrigger myTrigger = buildEventTrigger();}
 * <li>Create the state machine:<br>
 * {@code StateMachine<StateEnum, State> myStateMachine = factory.build(StateEnum.A);}
 * </ol>
 * </p>
 * 
 * @author Sylvain
 *
 * @param <K> Type of {@link Enum} that lists the potential states.
 * @param <S> Type of {@link State} that is contained in the state machine.
 */
public class EventBasedStateMachineFactory<K extends Enum<K>, S extends EventState>
{
   private String namePrefix;
   private YoRegistry registry;

   private final Map<K, S> states;
   private final Map<K, StateTransition<K>> stateTransitions;
   private final Map<K, StateEventCallback> stateCallbacks;
   private final StateEventTriggers<K> stateEventTriggers;
   private final List<StateChangedListener<K>> stateChangedListeners = new ArrayList<>();
   private final AtomicReference<Object> eventFired = new AtomicReference<>(null);
   private StateMachine<K, S> stateMachine;
   private StateMachineClock clock = StateMachineClock.dummyClock();
   private final Class<K> keyType;

   /**
    * Creates a new factory ready to be used for configuring and creating a new {@link StateMachine}.
    * <p>
    * Once the factory created, do not forget to setup the name, registry, and clock, via
    * {@link #setNamePrefix(String)}, {@link #setRegistry(YoRegistry)}, and
    * {@link #buildYoClock(DoubleProvider)} respectively.
    * </p>
    * 
    * @param keyType the type of the key to use, it is mostly used to create {@code EnumMap}s and
    *           {@link StateTransition}s.
    */
   public EventBasedStateMachineFactory(Class<K> keyType)
   {
      this.keyType = keyType;
      states = new EnumMap<>(keyType);
      stateTransitions = new EnumMap<>(keyType);
      stateCallbacks = new EnumMap<>(keyType);
      stateEventTriggers = new StateEventTriggers<>(keyType);
   }

   /**
    * Sets the name prefix that is used for the state machine internal {@code YoVariable}s and the
    * {@code StateMachineClock} if created to be baked with {@code YoVariable}s.
    * 
    * @param namePrefix the name prefix to be used for all {@code YoVariable}s.
    * @return this factory for chaining operations.
    */
   public EventBasedStateMachineFactory<K, S> setNamePrefix(String namePrefix)
   {
      this.namePrefix = namePrefix;
      return this;
   }

   /**
    * Sets the registry to which all {@code YoVariable}s will be registered to.
    * 
    * @param registry the registry to add the {@code YoVariable}s.
    * @return this factory for chaining operations.
    */
   public EventBasedStateMachineFactory<K, S> setRegistry(YoRegistry registry)
   {
      this.registry = registry;
      return this;
   }

   /**
    * Sets up a clock for the state machine.
    * <p>
    * the clock is baked with doubles, for a clock baked with {@code YoVariable}s use
    * {@link #buildYoClock(DoubleProvider)}.
    * </p>
    * 
    * @param timeProvider the variable used to obtain the time information.
    * @return this factory for chaining operations.
    */
   public EventBasedStateMachineFactory<K, S> buildClock(DoubleProvider timeProvider)
   {
      clock = StateMachineClock.clock(timeProvider);
      return this;
   }

   /**
    * Sets up a clock for the state machine.
    * <p>
    * the clock is baked with {@code YoVariable}s, for a clock baked with doubles use
    * {@link #buildClock(DoubleProvider)}.
    * </p>
    * 
    * @param timeProvider the variable used to obtain the time information.
    * @return this factory for chaining operations.
    * @throws RuntimeException if the registry has not been set. The registry can be added via
    *            {@link #setRegistry(YoRegistry)}.
    */
   public EventBasedStateMachineFactory<K, S> buildYoClock(DoubleProvider timeProvider)
   {
      if (namePrefix == null || registry == null)
         throw new RuntimeException("The namePrefix and registry fields have to be set in order to create yo-variables.");
      clock = StateMachineClock.yoClock(timeProvider, namePrefix, registry);
      return this;
   }

   /**
    * Registers a new state with its corresponding key.
    * 
    * @param key the key to be associated with the new state.
    * @param state the new state.
    * @return this factory for chaining operations.
    */
   public EventBasedStateMachineFactory<K, S> addState(K key, S state)
   {
      S oldState = states.put(key, state);
      if (oldState != null)
         PrintTools.warn("The state " + oldState.getClass().getSimpleName() + " at the key " + key + " has been replaced with "
               + state.getClass().getSimpleName());

      stateEventTriggers.addState(key, state);

      return this;
   }

   /**
    * Tests if there is state that has been registered for the given key.
    * 
    * @param stateKey the key to look for.
    * @return {@code true} is a state has been registered with the given key, {@code false} otherwise.
    */
   public boolean isStateRegistered(K stateKey)
   {
      return states.containsKey(stateKey);
   }

   /**
    * Creates a set of all the state keys that already have been registered.
    * 
    * @return the set keys corresponding to all the registered states.
    */
   public Set<K> getRegisteredStateKeys()
   {
      return states.keySet();
   }

   /**
    * Creates a transition from the source state {@code from} to the target state {@code to} that will
    * trigger when the source is active and the given {@code event} is fired at the state machine.
    * 
    * @param event the event to respond to.
    * @param from the key of the source state from which a transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @return this factory for chaining operations.
    */
   public <E extends Enum<E>> EventBasedStateMachineFactory<K, S> addTransition(E event, K from, K to)
   {
      addTransition(from, to, time -> eventFired.get() == event);
      return this;
   }

   /**
    * Register a runnable to callback when the given {@code event} is fired and that the state mapped
    * to {@code stateKey} is active.
    * <p>
    * A callback does not imply that a state transition is performed.
    * </p>
    * 
    * @param event the event to respond to.
    * @param stateKey the key of the state that should be active to perform the callback.
    * @param callback the runnable used to perform the callback.
    * @return this factory for chaining operations.
    */
   public <E extends Enum<E>> EventBasedStateMachineFactory<K, S> addCallback(E event, K stateKey, Runnable callback)
   {
      StateEventCallback callbacks = stateCallbacks.get(stateKey);
      if (callbacks == null)
      {
         callbacks = new StateEventCallback();
         stateCallbacks.put(stateKey, callbacks);
      }
      callbacks.registerCallback(event, callback);
      return this;
   }

   private EventBasedStateMachineFactory<K, S> addTransition(K from, K to, StateTransitionCondition condition)
   {
      StateTransition<K> stateTransition = stateTransitions.get(from);

      if (stateTransition == null)
      {
         stateTransition = new StateTransition<>(keyType);
         stateTransitions.put(from, stateTransition);
      }

      stateTransition.addCondition(to, condition);

      return this;
   }

   /**
    * Registers a listener that is to be used with the new state machine.
    * 
    * @param stateChangeListener the listener to be registered.
    * @return this factory for chaining operations.
    */
   public EventBasedStateMachineFactory<K, S> addStateChangedListener(StateChangedListener<K> stateChangeListener)
   {
      stateChangedListeners.add(stateChangeListener);
      return this;
   }

   /**
    * Instantiate the state machine.
    * <p>
    * Before calling this method, make sure you have provided: a name, registry, and clock via
    * {@link #setNamePrefix(String)}, {@link #setRegistry(YoRegistry)}, and
    * {@link #buildYoClock(DoubleProvider)} respectively.
    * </p>
    * 
    * @param initialStateKey the key of the very first state the state machine will enter.
    * @return the new state machine ready for use.
    */
   public StateMachine<K, S> build(K initialStateKey)
   {
      stateMachine = new StateMachine<K, S>(initialStateKey, states, stateTransitions, stateChangedListeners, clock, namePrefix, registry);

      // Get the current state event
      stateMachine.addPreTransitionCallback(() -> {
         K currentStateKey = stateMachine.getCurrentStateKey();
         if (currentStateKey != null)
         {
            Object newEvent = stateEventTriggers.fireEvent(currentStateKey, stateMachine.getTimeInCurrentState());
            if (newEvent != null && eventFired.get() == null)
               eventFired.set(newEvent);
         }
      });

      stateMachine.addPreTransitionCallback(() -> {
         StateEventCallback callbacks = stateCallbacks.get(stateMachine.getCurrentStateKey());
         if (callbacks != null)
            callbacks.processEvent(eventFired.get());
      });

      // Consume the event
      stateMachine.addPostTransitionCallback(() -> eventFired.set(null));

      return stateMachine;
   }

   /**
    * Creates a new event trigger that can be used to fire events at the state machine from anywhere.
    * 
    * @return the new event trigger.
    */
   public EventTrigger buildEventTrigger()
   {
      return eventFired::set;
   }

   /**
    * Creates a new event trigger to fire events at the state machine from a {@code YoVariable}
    * visualizer.
    * <p>
    * This trigger is meant to be used with a GUI in which {@code YoVariable}s can be changed.
    * </p>
    * <p>
    * An event is fired every time the {@code YoEnum} changes value.
    * </p>
    * 
    * @param triggerName the name of the new {@code YoEnum} to create.
    * @param eventType the type of event that can be fired.
    * @return this factory for chaining operations.
    */
   public <E extends Enum<E>> EventBasedStateMachineFactory<K, S> buildYoEventTrigger(String triggerName, Class<E> eventType)
   {
      if (registry == null)
         throw new RuntimeException("The registry field has to be set in order to create yo-variables.");
      YoEnum<E> yoTrigger = new YoEnum<>(triggerName, registry, eventType, true);
      yoTrigger.set(null);
      return buildYoEventTrigger(yoTrigger);
   }

   /**
    * Creates a new event trigger from the given {@code yoTrigger}.
    * <p>
    * An event is fired at the state machine every time {@code yoTrigger} changes value.
    * </p>
    * 
    * @param yoTrigger the variable to use for firing events.
    * @return this factory for chaining operations.
    */
   public <E extends Enum<E>> EventBasedStateMachineFactory<K, S> buildYoEventTrigger(YoEnum<E> yoTrigger)
   {
      EventTrigger eventTrigger = buildEventTrigger();

      yoTrigger.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            E newEvent = yoTrigger.getEnumValue();
            if (newEvent != null)
            {
               eventTrigger.fireEvent(yoTrigger.getEnumValue());
               yoTrigger.set(null);
            }
         }
      });
      return this;
   }

   private static class StateEventCallback
   {
      private final Map<Object, List<Runnable>> eventToCallbacksMap = new HashMap<>();;

      public void registerCallback(Object event, Runnable callback)
      {
         List<Runnable> callbacks = eventToCallbacksMap.get(event);
         if (callbacks == null)
         {
            callbacks = new ArrayList<>();
            eventToCallbacksMap.put(event, callbacks);
         }
         callbacks.add(callback);
      }

      public void processEvent(Object event)
      {
         List<Runnable> callbacks = eventToCallbacksMap.get(event);
         if (callbacks == null)
            return;

         for (int i = 0; i < callbacks.size(); i++)
            callbacks.get(i).run();
      }
   }

   private static class StateEventTriggers<K extends Enum<K>>
   {
      private final Map<K, EventState> states;

      public StateEventTriggers(Class<K> keyType)
      {
         states = new EnumMap<>(keyType);
      }

      public void addState(K stateKey, EventState state)
      {
         states.put(stateKey, state);
      }

      public Object fireEvent(K currentStateKey, double timeInState)
      {
         return states.get(currentStateKey).fireEvent(timeInState);
      }
   }
}
