package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.util.ArrayList;

public abstract class State<E extends Enum<E>>
{
   private final E stateEnum;
   private ArrayList<StateTransition<E>> stateTransitions = new ArrayList<StateTransition<E>>();
   private StateTransition<E> defaultNextStateTransition;
   private boolean gotoDefaultNextState;
   private TimeInCurrentStateProvider timeInCurrentStateProvider;
   private PreviousStateProvider<E, State<E>> previousStateProvider;

   public State(E stateEnum)
   {
      this.stateEnum = stateEnum;
   }

   // These are abstract methods that a state defines.
   public abstract void doAction();

   public abstract void doTransitionIntoAction();

   public abstract void doTransitionOutOfAction();

   public final void addStateTransition(E nextStateEnum, StateTransitionCondition stateTransitionCondition)
   {
      StateTransition<E> stateTransition = new StateTransition<E>(nextStateEnum, stateTransitionCondition, (StateTransitionAction) null);
      this.addStateTransition(stateTransition);
   }

   public final void addStateTransition(StateTransition<E> stateTransition)
   {
      stateTransitions.add(stateTransition);
   }

   public final void addDelayBasedStateTransition(final E nextStateEnum, final double delay)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return (getTimeInCurrentState() > delay);
         }
      };

      stateTransitions.add(new StateTransition<E>(nextStateEnum, stateTransitionCondition));
   }

   public final void addImmediateStateTransition(final E nextStateEnum)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return true;
         }
      };

      stateTransitions.add(new StateTransition<E>(nextStateEnum, stateTransitionCondition));
   }

   public final void setDefaultNextState(E nextStateEnum)
   {
      if (defaultNextStateTransition != null)
         throw new RuntimeException("Have already set default next state for " + stateEnum);
      defaultNextStateTransition = new StateTransition<E>(nextStateEnum, (StateTransitionCondition) null);
   }

   public final StateTransition<E> getDefaultNextStateTransition()
   {
      return defaultNextStateTransition;
   }

   public final void transitionToDefaultNextState()
   {
      gotoDefaultNextState = true;
   }

   public final double getTimeInCurrentState()
   {
      return timeInCurrentStateProvider.timeInCurrentState();
   }

   public final ArrayList<StateTransition<E>> getStateTransitions()
   {
      return stateTransitions;
   }

   public final E getStateEnum()
   {
      if(stateEnum == null)
         throw new RuntimeException("A state has not had its state enume set stateEnum is null for this state.");
      return stateEnum;
   }

   public final State<E> getPreviousState()
   {
      return previousStateProvider.getPreviousState();
   }

   public String toString()
   {
      StringBuffer stringBuffer = new StringBuffer();

      stringBuffer.append(stateEnum + ": (");

      boolean firstTime = true;
      for (StateTransition<E> stateTransition : stateTransitions)
      {
         if (!firstTime)
            stringBuffer.append(", ");
         stringBuffer.append(stateTransition.getNextStateEnum());
         firstTime = false;
      }

      stringBuffer.append(")");

      return stringBuffer.toString();
   }

   // Default methods. User should not use these. Only executor of a state machine should use these.
   final boolean getTransitionToDefaultNextState()
   {
      return gotoDefaultNextState;
   }

   final void setTimeInCurrentStateProvider(TimeInCurrentStateProvider timeInCurrentStateProvider)
   {
      this.timeInCurrentStateProvider = timeInCurrentStateProvider;
   }

   final void setPreviousStateProvider(PreviousStateProvider<E, State<E>> provider)
   {
      this.previousStateProvider = provider;
   }

   final void clearTransitionToDefaultNextState()
   {
      gotoDefaultNextState = false;
   }

   StateTransition<E> checkTransitionConditions(double timeInState)
   {
      for (int i = 0; i < stateTransitions.size(); i++)
      {
         StateTransition<E> stateTransition = stateTransitions.get(i);

         E nextStateEnum = stateTransition.checkTransitionConditions(timeInState);
         if (nextStateEnum != null)
         {
            return stateTransition;
         }
      }

      return null;
   }
}
