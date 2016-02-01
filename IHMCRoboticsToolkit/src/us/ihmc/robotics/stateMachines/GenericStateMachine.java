package us.ihmc.robotics.stateMachines;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;


public class GenericStateMachine<E extends Enum<E>, T extends State<E>> implements TimeInCurrentStateProvider
{
   private static final boolean DEBUG = false;
   private final String name;
   private final EnumYoVariable<E> stateYoVariable;
   private final DoubleYoVariable switchTimeYoVariable, t;
   private ArrayList<StateChangedListener<E>> stateChangedListeners;

   protected ArrayList<T> states = new ArrayList<T>();

   private T cachedCurrentState;

   public GenericStateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable t, YoVariableRegistry registry)
   {
      this.name = name;
      stateYoVariable = new EnumYoVariable<E>(name, registry, enumType);
      switchTimeYoVariable = new DoubleYoVariable(switchTimeName, registry);
      this.t = t;
      switchTimeYoVariable.set(t.getDoubleValue());
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
      states.add(state);
      if (cachedCurrentState == null)
         cachedCurrentState = state;
   }

   public void setCurrentState(E nextStateEnum)
   {
      if (DEBUG)
         System.out.println(name + ": t = " + t.getDoubleValue() + ": going to state: " + nextStateEnum.toString());

      T previousState = getCurrentState();

      switchTimeYoVariable.set(t.getDoubleValue());
      stateYoVariable.set(nextStateEnum);

      for (int i = 0; i < states.size(); i++)
      {
         T state = states.get(i);
         if (state.getStateEnum() == nextStateEnum)
         {
            if (stateChangedListeners != null)
            {
               for (StateChangedListener<E> listener : stateChangedListeners)
               {
                  listener.stateChanged(cachedCurrentState, state, switchTimeYoVariable.getDoubleValue());
               }
            }

            cachedCurrentState = state;

            state.setPreviousState(previousState);
            state.doTransitionIntoAction();

            return;
         }
      }

      throw new RuntimeException("Need to add state " + nextStateEnum + " to the state machine. Can't transition into the state unless it is added!");
   }

   public boolean isCurrentState(E stateEnum)
   {
      return (stateYoVariable.getEnumValue() == stateEnum);
   }

   public double timeInCurrentState()
   {
      return t.getDoubleValue() - switchTimeYoVariable.getDoubleValue();
   }

   public boolean inCurrentStateForDuration(double duration)
   {
      return (timeInCurrentState() >= duration);
   }

   public void doAction()
   {
      T currentState = getAndCheckCurrentState();
      currentState.doAction();
   }

   public T getCurrentState()
   {
      if (stateYoVariable.getEnumValue() == cachedCurrentState.getStateEnum())
         return cachedCurrentState;

      for (T state : states)
      {
         if (isCurrentState(state.getStateEnum()))
         {
            cachedCurrentState = state;

            return state;
         }
      }

      return null;
   }

   /**
    * Same as getCurrentState() except that it throws an RuntimeException if the current state enum 
    * @return
    */
   public T getAndCheckCurrentState()
   {
      T currentState = getCurrentState();

      if (currentState == null)
      {
         throw new RuntimeException("You forgot to add " + getCurrentStateEnum() + " to the state Machine.");
      }

      return currentState;
   }

   public E getCurrentStateEnum()
   {
      return getCurrentState().getStateEnum();
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

   public void checkTransitionConditions()
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
            return;
         }
      }

      // Make a Transition. First do the old states doTransitionOutOfAction, then do the stateTransition do Action.
      // Then set the current state to the new state which will also do the new states doTransitionIntoAction method.

      currentState.doTransitionOutOfAction();
      stateTransition.doAction();
      setCurrentState(stateTransition.nextStateEnum);
   }

   public void checkTransitionConditionsThoroughly()
   {
      E oldState;
      E newState;
      do
      {
         oldState = getCurrentStateEnum();
         checkTransitionConditions();
         newState = getCurrentStateEnum();
      }
      while (newState != oldState);
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
