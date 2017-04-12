package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

/**
 * GenericStateMachine. Class for construction a finite state machine. Requires an Enum, which lists all the possible state names.
 * An EnumYoVariable keeps track of the current state and a DoubleYoVariable keeps track of the time since the state was switched.
 * A GenericStateMachine is not dynamic. New states cannot be constructed on the fly.
 * The user is given the option to set the initial state in the constructor.
 * If they do not set an initialState, then it defaults to the first state in the Enum list.
 * Setting the initialState does not call the doTransitionIntoAction of that state.
 * In order to call the doTransitionIntoAction the user must call setCurrentState() instead of construct with an initial state.
 *
 * @param <E> Type of Enum that lists the potential states.
 * @param <T> Type of State that is contained in the state machine.
 */
public class GenericStateMachine<E extends Enum<E>, T extends State<E>> implements TimeInCurrentStateProvider, PreviousStateProvider<E, State<E>>
{
   private static final boolean DEBUG = false;

   private final EnumMap<E, T> enumsToStates;

   private final EnumYoVariable<E> stateYoVariable, previousStateYoVariable;
   private final DoubleYoVariable switchTimeYoVariable;
   private final DoubleProvider time;
   private ArrayList<StateChangedListener<E>> stateChangedListeners;

   protected ArrayList<T> states = new ArrayList<T>();

   public GenericStateMachine(String name, String switchTimeName, Class<E> enumType, E initialState, DoubleYoVariable timeVariable, YoVariableRegistry registry)
   {
      this(name, switchTimeName, enumType, initialState, new YoVariableDoubleProvider(timeVariable), registry);
   }

   public GenericStateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable timeVariable, YoVariableRegistry registry)
   {
      this(name, switchTimeName, enumType, null, new YoVariableDoubleProvider(timeVariable), registry);
   }

   public GenericStateMachine(String stateYoVariableName, String switchTimeName, Class<E> enumType, DoubleProvider timeProvider, YoVariableRegistry registry)
   {
      this(stateYoVariableName, switchTimeName, enumType, null, timeProvider, registry);
   }

   public GenericStateMachine(String stateYoVariableName, String switchTimeName, Class<E> enumType, E initialState, DoubleProvider timeProvider,
         YoVariableRegistry registry)
   {
      stateYoVariable = new EnumYoVariable<E>(stateYoVariableName, "State machine variable to keep track of the state.", registry, enumType, false);
      previousStateYoVariable = new EnumYoVariable<E>(stateYoVariableName + "PreviousState", "State machine variable to keep track of the previous state.",
            registry, enumType, true);

      enumsToStates = new EnumMap<>(enumType);

      if (initialState != null)
      {
         stateYoVariable.set(initialState);
      }

      previousStateYoVariable.set(null);

      switchTimeYoVariable = new DoubleYoVariable(switchTimeName, registry);
      this.time = timeProvider;
      switchTimeYoVariable.set(time.getValue());
   }

   public String getStateYoVariableName()
   {
      return stateYoVariable.getName();
   }

   public String getSwitchTimeName()
   {
      return switchTimeYoVariable.getName();
   }

   public void attachStateChangedListener(StateChangedListener<E> listener)
   {
      if (stateChangedListeners == null)
         stateChangedListeners = new ArrayList<StateChangedListener<E>>();
      stateChangedListeners.add(listener);
   }

   public void addState(T state)
   {
      for (T tempState : states)
      {
         if (tempState.getStateEnum() == state.getStateEnum())
            throw new RuntimeException("Duplicate state enums, name: " + state.getStateEnum() + ", already in use.");
      }
      state.setTimeInCurrentStateProvider(this);
      state.setPreviousStateProvider(this);
      states.add(state);
      enumsToStates.put(state.getStateEnum(), state);
   }

   public void setCurrentState(E nextStateEnum)
   {
      if (DEBUG)
         System.out.println(getStateYoVariableName() + ": t = " + time.getValue() + ": going to state: " + nextStateEnum.toString());

      T previousState = getCurrentState();
      T state = enumsToStates.get(nextStateEnum);

      if (state != null)
      {
         switchTimeYoVariable.set(time.getValue());
         previousStateYoVariable.set(stateYoVariable.getEnumValue());
         stateYoVariable.set(nextStateEnum);

         if (stateChangedListeners != null)
         {
            for (StateChangedListener<E> listener : stateChangedListeners)
            {
               listener.stateChanged(previousState, state, switchTimeYoVariable.getDoubleValue());
            }
         }
         state.doTransitionIntoAction();
      }

      else
      {
         throw new RuntimeException("Need to add state " + nextStateEnum + " to the state machine. Can't transition into the state unless it is added!");
      }
   }

   public boolean isCurrentState(E stateEnum)
   {
      return (stateYoVariable.getEnumValue() == stateEnum);
   }

   public double timeInCurrentState()
   {
      return time.getValue() - switchTimeYoVariable.getDoubleValue();
   }

   public boolean inCurrentStateForDuration(double duration)
   {
      return (timeInCurrentState() >= duration);
   }

   public void doAction()
   {
      T currentState = getCurrentState();
      if (currentState != null)
         currentState.doAction();
   }

   public T getCurrentState()
   {
      E stateEnum = stateYoVariable.getEnumValue();
      return enumsToStates.get(stateEnum);
   }

   public T getPreviousState()
   {
      E stateEnum = previousStateYoVariable.getEnumValue();
      return enumsToStates.get(stateEnum);
   }

   public E getCurrentStateEnum()
   {
      return stateYoVariable.getEnumValue();
   }

   public E getPreviousStateEnum()
   {
      E stateEnum = previousStateYoVariable.getEnumValue();
      return stateEnum;
   }

   public T getState(E stateEnum)
   {
      for (T state : states)
      {
         if (state.getStateEnum() == stateEnum)
            return state;
      }

      return null;
   }

   public boolean checkTransitionConditions()
   {
      T currentState = getCurrentState();

      StateTransition<E> stateTransition = currentState.checkTransitionConditions(timeInCurrentState());
      if (stateTransition == null)
      {
         if (currentState.getTransitionToDefaultNextState())
         {
            currentState.clearTransitionToDefaultNextState();
            stateTransition = currentState.getDefaultNextStateTransition();

            if (stateTransition == null)
            {
               throw new RuntimeException("DefaultNextState was not set for currentState=" + currentState);
            }
         }
         else
         {
            return false;
         }
      }

      // Make a Transition. First do the old states doTransitionOutOfAction, then do the stateTransition do Action.
      // Then set the current state to the new state which will also do the new states doTransitionIntoAction method.

      currentState.doTransitionOutOfAction();
      stateTransition.doAction();
      setCurrentState(stateTransition.getNextStateEnum());
      return true;
   }

   public String toString()
   {
      StringBuffer stringBuffer = new StringBuffer();

      // First print the states:
      stringBuffer.append("State Machine:\n");

      //    for (State state : states)
      //    {
      //       stringBuffer.append(state.getStateEnum());
      //       stringBuffer.append("\n");
      //    }

      for (T state : states)
      {
         stringBuffer.append(state.toString());
         stringBuffer.append("\n");
      }

      return stringBuffer.toString();
   }

   public EnumYoVariable<E> getStateYoVariable()
   {
      return stateYoVariable;
   }
}
