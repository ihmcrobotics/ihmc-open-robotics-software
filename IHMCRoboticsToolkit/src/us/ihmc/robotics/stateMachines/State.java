package us.ihmc.robotics.stateMachines;

import java.util.ArrayList;



public abstract class State <E extends Enum<E>>
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

   public abstract void doAction();

   public abstract void doTransitionIntoAction();

   public abstract void doTransitionOutOfAction();

   void setTimeInCurrentStateProvider(TimeInCurrentStateProvider timeInCurrentStateProvider)
   {
      this.timeInCurrentStateProvider = timeInCurrentStateProvider;
   }
   
   public void addStateTransition(StateTransition<E> stateTransition)
   {
      stateTransitions.add(stateTransition);
   }
   
   public void addDelayBasedStateTransition(final E nextStateEnum,final double delay)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {        
         @Override
         public boolean checkCondition()
         {
            return (getTimeInCurrentState() > delay );                 
         }
      };
      
      stateTransitions.add (new StateTransition<E>(nextStateEnum, stateTransitionCondition ) );
   }
   
   public void addImmediateStateTransition(final E nextStateEnum)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {        
         @Override
         public boolean checkCondition()
         {
            return  true;        
         }
      };
      
      stateTransitions.add (new StateTransition<E>(nextStateEnum, stateTransitionCondition ) );
   }

   public void setDefaultNextState(E nextStateEnum)
   {
      if (defaultNextStateTransition != null) throw new RuntimeException("Have already set default next state for " + stateEnum);
      defaultNextStateTransition = new StateTransition<E>(nextStateEnum, (StateTransitionCondition) null);
   }


   public StateTransition<E> getDefaultNextStateTransition()
   {
      return defaultNextStateTransition;
   }

   public boolean getTransitionToDefaultNextState()
   {
      return gotoDefaultNextState;
   }

   public void clearTransitionToDefaultNextState()
   {
      gotoDefaultNextState = false;
   }

   public void transitionToDefaultNextState()
   {
      gotoDefaultNextState = true;
   }


   public StateTransition<E> checkTransitionConditions(double timeInState)
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

   public double getTimeInCurrentState()
   {
      return timeInCurrentStateProvider.timeInCurrentState();
   }

   public ArrayList<StateTransition<E>> getStateTransitions()
   {
      return stateTransitions;
   }

   public E getStateEnum()
   {
      return stateEnum;
   }
   
   public void setPreviousState(State<E> previousState)
   {
      this.previousState = previousState;
   }
   
   public State<E> getPreviousState()
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
   
   public boolean isDone()
   {
      return true;
   }
}
