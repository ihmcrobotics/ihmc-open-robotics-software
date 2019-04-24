package us.ihmc.robotics.stateMachine.factories;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * This factory gathers convenience methods to configure and create a {@link StateMachine}.
 * <p>
 * Here is an example for creating a simple state machine that represents the main usecase of this
 * factory:
 * <ol>
 * <li>Create new factory: <br>
 * {@code StateMachineFactory<StateEnum, State> factory = new StateMachineFactory<>(StateEnum.class);}
 * <li>Setup the name, registry, and clock:<br>
 * {@code factory.setNamePrefix("myStateMachine").setRegistry(myRegistry).buildYoClock(yoTime);}
 * <li>Add the states:<br>
 * {@code factory.addState(StateEnum.A, stateA).addState(StateEnum.B, stateB);}
 * <li>Add the transitions:<br>
 * {@code factory.addTransition(StateEnum.A, StateEnum.B, time -> time > timeToStayInStateA);}
 * <li>Create the state machine:<br>
 * {@code StateMachine<StateEnum, State> myStateMachine = factory.build(StateEnum.A);}
 * </ol>
 * </p>
 * <p>
 * Please take a look at the factory which provides a different way of configuring a state machine:
 * {@link EventBasedStateMachineFactory}.
 * </p>
 * 
 * @author Sylvain
 *
 * @param <K> Type of {@link Enum} that lists the potential states.
 * @param <S> Type of {@link State} that is contained in the state machine.
 */
public class StateMachineFactory<K extends Enum<K>, S extends State>
{
   private String namePrefix = "stateMachine";
   private YoVariableRegistry registry;

   protected final Map<K, S> states;
   private final Map<K, StateTransition<K>> stateTransitions;
   private final List<StateChangedListener<K>> stateChangedListeners = new ArrayList<>();
   private StateMachineClock clock = StateMachineClock.dummyClock();
   private final Class<K> keyType;

   /**
    * Creates a new factory ready to be used for configuring and creating a new {@link StateMachine}.
    * <p>
    * Once the factory created, do not forget to setup the name, registry, and clock, via
    * {@link #setNamePrefix(String)}, {@link #setRegistry(YoVariableRegistry)}, and
    * {@link #buildYoClock(DoubleProvider)} respectively.
    * </p>
    * 
    * @param keyType the type of the key to use, it is mostly used to create {@code EnumMap}s and
    *           {@link StateTransition}s.
    */
   public StateMachineFactory(Class<K> keyType)
   {
      this.keyType = keyType;
      states = new EnumMap<>(keyType);
      stateTransitions = new EnumMap<>(keyType);
   }

   /**
    * Sets the name prefix that is used for the state machine internal {@code YoVariable}s and the
    * {@code StateMachineClock} if created to be baked with {@code YoVariable}s.
    * 
    * @param namePrefix the name prefix to be used for all {@code YoVariable}s.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> setNamePrefix(String namePrefix)
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
   public StateMachineFactory<K, S> setRegistry(YoVariableRegistry registry)
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
   public StateMachineFactory<K, S> buildClock(DoubleProvider timeProvider)
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
    *            {@link #setRegistry(YoVariableRegistry)}.
    */
   public StateMachineFactory<K, S> buildYoClock(DoubleProvider timeProvider)
   {
      if (namePrefix == null || registry == null)
         throw new RuntimeException("The namePrefix and registry fields have to be set in order to create yo-variables. namePrefix:"+namePrefix+" registry:"+registry);
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
   public StateMachineFactory<K, S> addState(K key, S state)
   {
      S oldState = states.put(key, state);
      if (oldState != null)
         PrintTools.warn("The state " + oldState.getClass().getSimpleName() + " at the key " + key + " has been replaced with "
               + state.getClass().getSimpleName());
      return this;
   }

   /**
    * Registers a new state with its corresponding key and creates a transition that will trigger when
    * this new state is active and its method {@link State#isDone(double)} returns {@code true}.
    * <p>
    * This method is very easy to use as the only requirement is implement the method
    * {@link State#isDone(double)} in the given state. A state machine be entirely configured using
    * this method only. The transition created here is compatible with the other types, such that after
    * calling this method, the state machine can still be configured with additional transitions
    * starting off this new state.
    * </p>
    * 
    * @param key the key to be associated with the new state.
    * @param state the new state.
    * @param nextStateKey the key of the next state to be reached once the state will be active and
    *           done.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addStateAndDoneTransition(K key, S state, K nextStateKey)
   {
      addState(key, state);
      addDoneTransition(key, nextStateKey);
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
    * Creates a set of all the state keys that already have been registered.
    * 
    * @return the set keys corresponding to all the registered states.
    */
   public Collection<S> getRegisteredStates()
   {
      return states.values();
   }

   /**
    * Creates a transition from the source state {@code from} to the target state {@code to} that will
    * trigger when the source is active and is done, i.e. {@code State#isDone(double)} returns
    * {@code true}.
    * <p>
    * For this transition to be effective, the source state <b>must</b> override the method
    * {@link State#isDone(double)}.
    * </p>
    * 
    * @param from the key of the source state from which a transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addDoneTransition(K from, K to)
   {
      return addTransition(from, to, states.get(from)::isDone);
   }

   /**
    * Creates a set of requestable transitions from the source state {@code from} to all the
    * <b>registered</b> states except {@code from}.
    * <p>
    * When the state {@code from} is active, then a transition to any of the other states can be
    * triggered simply by setting the value of {@code requestedState} to the desired target state.When
    * the transition is triggered, the value of {@code requestedState} is set to {@code null} to notify
    * that the transition happened.
    * </p>
    * 
    * @param from the key of the source state from which a transition can happen.
    * @param requestedState the variable providing the desired target state when a transition is to be
    *           triggered.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addRequestedTransition(K from, YoEnum<K> requestedState)
   {
      return addRequestedTransition(from, requestedState, false);
   }

   /**
    * Creates a transition from the source state {@code from} to the target state {@code to}
    * requestable from the given {@code requestedState}.
    * <p>
    * The transition is triggered when {@code from} is the active state and
    * {@code requestedState.getEnumValue() == to}. When the transition is triggered, the value of
    * {@code requestedState} is set to {@code null} to notify that the transition happened.
    * </p>
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @param requestedState the trigger for activating the transition.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addRequestedTransition(K from, K to, YoEnum<K> requestedState)
   {
      return addRequestedTransition(from, to, requestedState, false);
   }

   /**
    * Creates a set of requestable transitions from the source state {@code from} to all the
    * <b>registered</b> states except {@code from}.
    * <p>
    * When the state {@code from} is active, then a transition to any of the other states can be
    * triggered simply by setting the value of {@code requestedState} to the desired target state.When
    * the transition is triggered, the value of {@code requestedState} is set to {@code null} to notify
    * that the transition happened.
    * </p>
    * 
    * @param from the key of the source state from which a transition can happen.
    * @param requestedState the variable providing the desired target state when a transition is to be
    *           triggered.
    * @param waitUntilDone when {@code true} the transition will only be allowed to happen when the
    *           source state is done, i.e. {@code State#isDone(double)} returns {@code true}. When
    *           {@code false} the transition will trigger immediately after {@code requestedState} has
    *           the proper value.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addRequestedTransition(K from, YoEnum<K> requestedState, boolean waitUntilDone)
   {
      for (K to : getRegisteredStateKeys())
      {
         if (to != from)
            addRequestedTransition(from, to, requestedState, waitUntilDone);
      }
      return this;
   }

   /**
    * Creates a transition from the source state {@code from} to the target state {@code to}
    * requestable from the given {@code requestedState}.
    * <p>
    * The transition is triggered when {@code from} is the active state and
    * {@code requestedState.getEnumValue() == to}. When the transition is triggered, the value of
    * {@code requestedState} is set to {@code null} to notify that the transition happened.
    * </p>
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @param requestedState the trigger for activating the transition.
    * @param waitUntilDone when {@code true} the transitions will only be allowed to happen when the
    *           source state is done, i.e. {@code State#isDone(double)} returns {@code true}. When
    *           {@code false} the transitions will trigger immediately after {@code requestedState} has
    *           the proper value.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addRequestedTransition(K from, K to, YoEnum<K> requestedState, boolean waitUntilDone)
   {
      return addTransition(from, to, new StateTransitionCondition()
      {
         @Override
         public boolean testCondition(double timeInCurrentState)
         {
            boolean ready = !waitUntilDone || states.get(from).isDone(timeInCurrentState);

            if (to == requestedState.getEnumValue() && ready)
            {
               requestedState.set(null);
               return true;
            }
            return false;
         }
      });
   }

   /**
    * Creates a time based transition from the source state {@code from} to the target state
    * {@code to}.
    * <p>
    * When the source state {@code from} is active, the transition into the target state {@code to} is
    * triggered as soon as the time spent in the source state is greater than
    * {@code durationBeforeTransition}.
    * </p>
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @param durationBeforeTransition the maximum time to spend in the source state.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addTimeBasedTransition(K from, K to, double durationBeforeTransition)
   {
      return addTransition(from, to, timeInCurrentState -> durationBeforeTransition <= timeInCurrentState);
   }

   /**
    * Creates a time based transition from the source state {@code from} to the target state
    * {@code to}.
    * <p>
    * When the source state {@code from} is active, the transition into the target state {@code to} is
    * triggered as soon as the time spent in the source state is greater than
    * {@code durationBeforeTransition}.
    * </p>
    * <p>
    * This is transition is identical to {@link #addTimeBasedTransition(Enum, Enum, double)}, however
    * it is allows the user to provide a maximum duration that dynamically changes.
    * </p>
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @param durationBeforeTransition the holder of the maximum time to spend in the source state.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addTimeBasedTransition(K from, K to, DoubleProvider durationBeforeTransition)
   {
      return addTransition(from, to, timeInCurrentState -> durationBeforeTransition.getValue() <= timeInCurrentState);
   }

   /**
    * Creates a transition from the source state {@code from} to the target state {@code to} that is
    * triggered as soon as the source state becomes active.
    * <p>
    * This type of transition is very rarely used as it will prevent the source state from running.
    * </p>
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addImmediateTransition(K from, K to)
   {
      return addTransition(from, to, timeInCurrentState -> true);
   }

   /**
    * Creates a set of transitions from all the given source states {@code froms} to the single target
    * state {@code to}.
    * <p>
    * When any of the source states becomes active, the condition, once fulfilled, will trigger a
    * transition to the unique target state.
    * </p>
    * 
    * @param froms the keys of all the source state from which the transitions can happen.
    * @param to the key of the target state to which the transitions will lead into.
    * @param condition the condition that will trigger any of the transitions.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addTransition(Iterable<? extends K> froms, K to, StateTransitionCondition condition)
   {
      froms.forEach(from -> addTransition(from, to, condition));
      return this;
   }

   /**
    * Creates a transition from the source state {@code from} to the target state {@code to} that is
    * triggered when the source state becomes active and the condition is fulfilled.
    * <p>
    * This is the most common way of creating state transitions as {@link StateTransitionCondition} is
    * very easy to implement and thus very flexible.
    * </p>
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param to the key of the target state to which the transition will lead into.
    * @param condition the condition that will trigger the transition.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addTransition(K from, K to, StateTransitionCondition condition)
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
    * Creates a transition from the source state {@code from} given a full state transition holding
    * both the conditions and their target states.
    * 
    * @param from the key of the source state from which the transition can happen.
    * @param stateTransition it carries the conditions associated with the keys of the target state.
    *           Its content is copied to a local copy.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addTransition(K from, StateTransition<K> stateTransition)
   {
      StateTransition<K> thisStateTransition = stateTransitions.get(from);

      if (thisStateTransition == null)
      {
         thisStateTransition = new StateTransition<>(keyType);
         stateTransitions.put(from, thisStateTransition);
      }

      thisStateTransition.completeWith(stateTransition);

      return this;
   }

   /**
    * Registers a listener that is to be used with the new state machine.
    * 
    * @param stateChangeListener the listener to be registered.
    * @return this factory for chaining operations.
    */
   public StateMachineFactory<K, S> addStateChangedListener(StateChangedListener<K> stateChangeListener)
   {
      stateChangedListeners.add(stateChangeListener);
      return this;
   }

   /**
    * Instantiate the state machine.
    * <p>
    * Before calling this method, make sure you have provided: a name, registry, and clock via
    * {@link #setNamePrefix(String)}, {@link #setRegistry(YoVariableRegistry)}, and
    * {@link #buildYoClock(DoubleProvider)} respectively.
    * </p>
    * 
    * @param initialStateKey the key of the very first state the state machine will enter.
    * @return the new state machine ready for use.
    */
   public StateMachine<K, S> build(K initialStateKey)
   {
      return new StateMachine<K, S>(initialStateKey, states, stateTransitions, stateChangedListeners, clock, namePrefix, registry);
   }
}
