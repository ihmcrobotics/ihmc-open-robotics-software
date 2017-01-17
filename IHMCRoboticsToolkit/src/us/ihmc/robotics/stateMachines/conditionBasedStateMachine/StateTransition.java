package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class StateTransition<E extends Enum<E>>
{
   private final E nextStateEnum;
   private final ArrayList<StateTransitionCondition> stateTransitionConditions = new ArrayList<StateTransitionCondition>();
   private ArrayList<DoubleYoVariable> timePassedYoVariables;
   private final ArrayList<StateTransitionAction> actions = new ArrayList<StateTransitionAction>();

   public StateTransition(E nextStateEnum, DoubleYoVariable timePassedYoVariable, StateTransitionCondition condition, StateTransitionAction action)
   {
      if (nextStateEnum == null)
         throw new RuntimeException("Cannot create StateTransition with null nextStateEnum!");

      this.nextStateEnum = nextStateEnum;
      if (condition != null)
         this.stateTransitionConditions.add(condition);
      if (action != null)
         this.actions.add(action);
      if (timePassedYoVariable != null)
         addTimePassedCondition(timePassedYoVariable);
   }

   public StateTransition(E nextStateEnum, StateTransitionCondition condition, StateTransitionAction action)
   {
      this(nextStateEnum, null, condition, action);
   }

   public StateTransition(E nextStateEnum, DoubleYoVariable timePassedYoVariable)
   {
      this(nextStateEnum, timePassedYoVariable, null, null);
   }

   public StateTransition(E nextStateEnum, StateTransitionCondition condition)
   {
      this(nextStateEnum, condition, (StateTransitionAction) null);
   }

   public StateTransition(E nextStateEnum, ArrayList<StateTransitionCondition> stateTransitionConditions, StateTransitionAction action)
   {
      this.nextStateEnum = nextStateEnum;
      this.stateTransitionConditions.addAll(stateTransitionConditions);
      if (action != null)
         this.actions.add(action);
   }

   public StateTransition(E nextStateEnum, StateTransitionCondition stateTransitionCondition, List<StateTransitionAction> actions)
   {
      this.nextStateEnum = nextStateEnum;
      if (stateTransitionCondition != null)
         this.stateTransitionConditions.add(stateTransitionCondition);
      this.actions.addAll(actions);
   }

   public StateTransition(E nextStateEnum, List<StateTransitionCondition> stateTransitionCondition, List<StateTransitionAction> actions)
   {
      this.nextStateEnum = nextStateEnum;
      this.stateTransitionConditions.addAll(stateTransitionCondition);
      this.actions.addAll(actions);
   }

   public StateTransition(E nextStateEnum, ArrayList<StateTransitionCondition> stateTransitionConditions)
   {
      this(nextStateEnum, stateTransitionConditions, (StateTransitionAction) null);
   }

   public void addStateTransitionCondition(StateTransitionCondition transitionCondition)
   {
      if (transitionCondition != null)
         stateTransitionConditions.add(transitionCondition);
   }

   public void addTimePassedCondition(DoubleYoVariable timePassedYoVariable)
   {
      if (timePassedYoVariables == null)
         timePassedYoVariables = new ArrayList<DoubleYoVariable>();
      if (timePassedYoVariable != null)
         timePassedYoVariables.add(timePassedYoVariable);
   }

   final E checkTransitionConditions(double timeInState)
   {
      for (int i = 0; i < stateTransitionConditions.size(); i++)
      {
         StateTransitionCondition stateTransitionCondition = stateTransitionConditions.get(i);
         if (!stateTransitionCondition.checkCondition())
         {
            return null;
         }
      }

      if (timePassedYoVariables == null)
         return nextStateEnum;

      for (DoubleYoVariable timePassedYoVariable : timePassedYoVariables)
      {
         if (timeInState < timePassedYoVariable.getDoubleValue())
         {
            return null;
         }
      }

      return nextStateEnum;
   }

   final void doAction()
   {
      for (int i = 0; i < actions.size(); i++)
      {
         actions.get(i).doTransitionAction();
      }
   }

   public E getNextStateEnum()
   {
      return nextStateEnum;
   }
}
