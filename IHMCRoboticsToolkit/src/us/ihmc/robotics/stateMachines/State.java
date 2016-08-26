package us.ihmc.robotics.stateMachines;

import java.util.ArrayList;

public abstract class State<E extends Enum<E>>
{
   private final E stateEnum;
   private State<E> previousState;
   private ArrayList<StateTransition<E>> stateTransitions = new ArrayList<StateTransition<E>>();
   private StateTransition<E> defaultNextStateTransition;
   private boolean gotoDefaultNextState = false;
   private TimeInCurrentStateProvider timeInCurrentStateProvider;

   public State(E stateEnum)
   {
      this.stateEnum = stateEnum;
   }

   // These are abstract methods that a state defines.
   public abstract void doAction();

   public abstract void doTransitionIntoAction();

   public abstract void doTransitionOutOfAction();

   /**
    * Override this method if you want to do a simple state machine with default transitions. This then becomes the default transition condition.
    * @return true if this state should transition to the default next state set by {@link #setDefaultNextState(Enum)}.
    */
   public boolean isDone()
   {
      return true;
   }

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
      return stateEnum;
   }

   public final State<E> getPreviousState()
   {
      return previousState;
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
         stringBuffer.append(stateTransition.nextStateEnum);
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

   final void clearTransitionToDefaultNextState()
   {
      gotoDefaultNextState = false;
   }

   final void setPreviousState(State<E> previousState)
   {
      this.previousState = previousState;
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
