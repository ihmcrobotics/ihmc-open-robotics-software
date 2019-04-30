package us.ihmc.robotics.stateMachine.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.stateMachine.extra.EventState;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * This class is an implementation of a finite state machine that contains a collection of
 * {@link State}s associated with {@link StateTransition}s.
 * <p>
 * To build a state machine, please use one the factories available:
 * <ul>
 * <li>{@link StateMachineFactory}: factory for simple state implementations.
 * <li>{@link EventBasedStateMachineFactory}: factory for creating a state machine which state
 * transitions are trigger by firing events. States must implement {@link EventState}.
 * </ul>
 * </p>
 * <p>
 * Once created, the states and transitions of a state machine are final, they cannot be modified.
 * </p>
 * 
 * @author Sylvain
 *
 * @param <K> Type of {@link Enum} that lists the potential states.
 * @param <S> Type of {@link State} that is contained in the state machine.
 */
public class StateMachine<K extends Enum<K>, S extends State>
{
   /**
    * Refers to the very first active state of this state machine and to the state that will be entered
    * upon calling {@link #reset()}.
    */
   private final K initialStateKey;
   /** The map of the states composing this state machine associated with their keys. */
   private final Map<K, S> states;
   /** The map of the state transitions associated with the state keys. */
   private final Map<K, StateTransition<K>> stateTransitions;
   /** The list of registered callbacks to run right before processing state transitions. */
   private final List<Runnable> preTransitionCallbacks = new ArrayList<>();
   /** The list of registered callbacks to run right after processing state transitions. */
   private final List<Runnable> postTransitionCallbacks = new ArrayList<>();
   /** The list of listeners to be called when the active state changes. */
   private final List<StateChangedListener<K>> stateChangedListeners;
   /** The internal clock of this state machine used especially to get the time spent in a state. */
   private final StateMachineClock clock;

   /** Reference to the active state. */
   private final YoEnum<K> currentStateKey;
   /** Reference to the previously active state. */
   private final YoEnum<K> previousStateKey;

   /**
    * Creates a new state machine.
    * <p>
    * The user should use one a the available factories to create a new state machine, see
    * {@link StateMachine}.
    * </p>
    * 
    * @param initialStateKey the key for the initial state. The state machine will transition into the
    *           initial state at the first call of {@link #doAction()} or when calling
    *           {@link #resetToInitialState()}.
    * @param states the map of the states composing this state machine associated with their keys.
    * @param stateTransitions the map of the state transitions associated with the state keys.
    * @param stateChangedListeners the list of listeners to be called when the active state changes.
    *           New listeners can be added later via
    *           {@link #addStateChangedListener(StateChangedListener)}.
    * @param clock the clock needed to provide the time spent in the active state.
    * @param namePrefix the prefix used when creating the {@code YoVariable}s for tracking the active
    *           and previous states.
    * @param registry the registry to which the {@code YoVariable}s in this state machine will be added
    *           to.
    */
   public StateMachine(K initialStateKey, Map<K, S> states, Map<K, StateTransition<K>> stateTransitions, List<StateChangedListener<K>> stateChangedListeners,
                       StateMachineClock clock, String namePrefix, YoVariableRegistry registry)
   {
      this.initialStateKey = initialStateKey;
      this.states = states;
      this.stateTransitions = stateTransitions;
      this.stateChangedListeners = stateChangedListeners;
      this.clock = clock;

      currentStateKey = new YoEnum<>(namePrefix + "CurrentState", "", registry, initialStateKey.getDeclaringClass(), true);
      currentStateKey.set(null);

      previousStateKey = new YoEnum<>(namePrefix + "PreviousState", "", registry, initialStateKey.getDeclaringClass(), true);
      previousStateKey.set(null);
      addStateChangedListener((oldKey, newKey) -> previousStateKey.set(oldKey));
   }

   /**
    * Registers a new listener that will be called when this state machine changes the active state.
    * 
    * @param listener the new listener to register.
    */
   public void addStateChangedListener(StateChangedListener<K> listener)
   {
      stateChangedListeners.add(listener);
   }

   /**
    * Registers a new callback that will be called right before state transitions are processed.
    * 
    * @param callback the callback to register.
    */
   public void addPreTransitionCallback(Runnable callback)
   {
      preTransitionCallbacks.add(callback);
   }

   /**
    * Registers a new callback that will be called right after state transitions are processed.
    * 
    * @param callback the callback to register.
    */
   public void addPostTransitionCallback(Runnable callback)
   {
      postTransitionCallbacks.add(callback);
   }

   /**
    * Resets this state machine back to its initial state.
    * <p>
    * The active state is exited immediately, calling first {@link State#onExit()}, and enters the
    * initial state of this state machine, i.e. the state mapped to the key
    * {@link #getInitialStateKey()}. This method will call {@link State#onEntry()} on the initial state
    * before activating it.
    * </p>
    * 
    * @throws RuntimeException if there is no state mapped to {@link #getInitialStateKey()}.
    */
   public void resetToInitialState()
   {
      performTransition(initialStateKey);
   }

   /**
    * Resets the active state.
    * <p>
    * More precisely, this method calls in order the active state's methods {@link State#onExit()} and
    * then {@link State#onEntry()}. The {@link StateChangedListener}s registered are also notified.
    * </p>
    */
   public void resetCurrentState()
   {
      performTransition(currentStateKey.getEnumValue());
   }

   /**
    * Processes requested transition, if any, and then calls {@link State#doAction(double)} on the
    * active state.
    * <p>
    * Always prefer using this method rather than calling independently {@link #doAction()} and
    * {@link #doTransitions()}.
    * </p>
    * <p>
    * On the very first call, the method ensures that {@link #resetToInitialState()} has been called at
    * least once. This ensures that the state machine starts with the initial state, i.e. the state
    * mapped to the key {@link #getInitialStateKey()}, as the active state.
    * </p>
    */
   public void doActionAndTransition()
   {
      doTransitions();
      doAction();
   }

   /**
    * Calls {@link State#doAction(double)} on the active state.
    * <p>
    * Always prefer using {@link #doActionAndTransition()} rather than calling independently
    * {@link #doAction()} and {@link #doTransitions()}.
    * </p>
    * <p>
    * On the very first call, the method ensures that {@link #resetToInitialState()} has been called at
    * least once. This ensures that the state machine starts with the initial state, i.e. the state
    * mapped to the key {@link #getInitialStateKey()}, as the active state.
    * </p>
    */
   public void doAction()
   {
      if (currentStateKey.getEnumValue() == null)
         resetToInitialState();

      S currentState = getState(currentStateKey.getEnumValue());
      assertStateNotNull(currentStateKey.getEnumValue(), currentState);

      currentState.doAction(clock.getTimeInCurrentState());
   }

   /**
    * Runs through the {@link StateTransition}s and performs requested transitions, if any.
    * <p>
    * Always prefer using {@link #doActionAndTransition()} rather than calling independently
    * {@link #doAction()} and {@link #doTransitions()}.
    * </p>
    * 
    * @return whether the active state is changing.
    * @throws RuntimeException if there is no state mapped to the key provided by the
    *            {@link StateTransition} of the active state.
    */
   public boolean doTransitions()
   {
      if (currentStateKey.getEnumValue() == null)
         resetToInitialState();

      callPreTransitionCallbacks();

      StateTransition<K> stateTransition = stateTransitions.get(currentStateKey.getEnumValue());

      
      if (stateTransition == null)
      {
         callPostTransitionCallbacks();
         return false;
      }

      K nextStateKey = stateTransition.isTransitionRequested(clock.getTimeInCurrentState());

      if (nextStateKey == null)
      {
         callPostTransitionCallbacks();
         return false;
      }

      performTransition(nextStateKey);

      callPostTransitionCallbacks();
      return true;
   }

   /**
    * Changes immediately the active state.
    * <p>
    * This method should be used cautiously and only for exceptional scenario, it is not the regular
    * way of using a state machine.
    * </p>
    * <p>
    * In order, this method calls {@link State#onExit()} on the state being exited, notifies the
    * {@link StateChangedListener}s registered, and then calls {@link State#onEntry()} on the new
    * active state.
    * </p>
    * 
    * @param nextStateKey the key of the state to transition into.
    * @throws RuntimeException if there is no state mapped to {@code nextStateKey}.
    */
   public void performTransition(K nextStateKey)
   {
      if (currentStateKey.getEnumValue() != null)
      {
         S currentState = states.get(currentStateKey.getEnumValue());
         if (currentState != null)
            currentState.onExit();
      }

      S nextState = getState(nextStateKey);
      assertStateNotNull(nextStateKey, nextState);

      if (stateChangedListeners != null)
      {
         for (int i = 0; i < stateChangedListeners.size(); i++)
            stateChangedListeners.get(i).stateChanged(currentStateKey.getEnumValue(), nextStateKey);
      }

      clock.notifyStateChanged();

      nextState.onEntry();
      currentStateKey.set(nextStateKey);
   }

   /**
    * Convenience method that calls of the registered pre-transition callbacks.
    */
   private void callPreTransitionCallbacks()
   {
      for (int i = 0; i < preTransitionCallbacks.size(); i++)
         preTransitionCallbacks.get(i).run();
   }

   /**
    * Convenience method that calls of the registered post-transition callbacks.
    */
   private void callPostTransitionCallbacks()
   {
      for (int i = 0; i < postTransitionCallbacks.size(); i++)
         postTransitionCallbacks.get(i).run();
   }

   /**
    * Gets the type of the key used for creating this state machine.
    * 
    * @return the key type.
    */
   public Class<K> getStateKeyType()
   {
      return initialStateKey.getDeclaringClass();
   }

   /**
    * Gets the key of the initial state.
    * 
    * @return the initial state key.
    */
   public K getInitialStateKey()
   {
      return initialStateKey;
   }

   /**
    * Gets the key of the active state.
    * 
    * @return the current state key.
    */
   public K getCurrentStateKey()
   {
      return currentStateKey.getEnumValue();
   }

   /**
    * Gets the key of the state that was previously active.
    * 
    * @return the previous state key.
    */
   public K getPreviousStateKey()
   {
      return previousStateKey.getEnumValue();
   }

   /**
    * Gets the active state.
    * 
    * @return the current state.
    * @throws RuntimeException if calling this method before either {@link #resetToInitialState()} or
    *            {@link #doAction()} was called at least once.
    */
   public S getCurrentState()
   {
      if (currentStateKey.getEnumValue() == null)
         throw new RuntimeException("StateMachine.reset() or doControl() has to be called before the current state key is initialized.");
      return getState(currentStateKey.getEnumValue());
   }

   /**
    * Gets the state that was previously active.
    *
    * @return the previous state.
    */
   public S getPreviousState()
   {
      return getState(getPreviousStateKey());
   }

   /**
    * Retrieves the state mapped to the given key.
    * 
    * @param stateKey the key of the state of interest.
    * @return the state corresponding to the key, or {@code null} if there is no state mapped to the
    *         key.
    */
   public S getState(K stateKey)
   {
      return states.get(stateKey);
   }

   /**
    * Retrieves the state transition associated to the given state key.
    * <p>
    * The returned transition is the only one used for processing transition requests when the active
    * state is mapped to {@code stateKey}.
    * </p>
    * 
    * @param stateKey the state key of the state transition of interest.
    * @return the state transition corresponding to the key, or {@code null} if there is no transition
    *         mapped to the key.
    */
   public StateTransition<K> getStateTransition(K stateKey)
   {
      return stateTransitions.get(stateKey);
   }

   /**
    * Gets the time spent since the last state change.
    * 
    * @return the time in the current state.
    */
   public double getTimeInCurrentState()
   {
      return clock.getTimeInCurrentState();
   }

   /**
    * Gets the absolute time when the last state change occurred.
    * 
    * @return the time of last state change.
    */
   public double getTimeOfLastStateChange()
   {
      return clock.getTimeOfLastStateChange();
   }

   /**
    * Tests if the active state is the last state that this state machine can enter, i.e. there is no
    * transition possible out of the active state.
    * <p>
    * This is usually useful to determine is this state machine is done.
    * </p>
    * 
    * @return {@code true} if there is no transition out of the current state, {@code false} otherwise.
    */
   public boolean isCurrentStateTerminal()
   {
      return !stateTransitions.containsKey(currentStateKey.getEnumValue());
   }

   private static void assertStateNotNull(Object stateKey, State state)
   {
      if (state == null)
         throw new RuntimeException("There is no state associated with the key: " + stateKey);
   }
}
