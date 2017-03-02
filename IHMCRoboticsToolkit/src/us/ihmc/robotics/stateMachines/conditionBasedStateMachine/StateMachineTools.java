package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;


/**
 * @author twan
 *         Date: 5/30/13
 */
public class StateMachineTools
{
   @SafeVarargs
   public static <T extends Enum<T>> void addRequestedStateTransition(final EnumYoVariable<T> requestedState, boolean waitUntilDone,
           final FinishableState<T> initialState, final FinishableState<T>... statePath)
   {
      addRequestedStateTransition(requestedState, waitUntilDone, new ArrayList<StateTransitionAction>(), initialState, statePath);
   }

   @SafeVarargs
   public static <T extends Enum<T>> void addRequestedStateTransition(final EnumYoVariable<T> requestedState, boolean waitUntilDone,
         StateTransitionAction stateTransitionAction, final FinishableState<T> initialState, final FinishableState<T>... statePath)
   {
      addRequestedStateTransition(requestedState, waitUntilDone, createListWithOneElement(stateTransitionAction), initialState, statePath);
   }

   @SafeVarargs
   public static <T extends Enum<T>> void addRequestedStateTransition(final EnumYoVariable<T> requestedState, boolean waitUntilDone,
           List<? extends StateTransitionAction> stateTransitionActions, final FinishableState<T> initialState, final FinishableState<T>... statePath)
   {
      final State<T> finalState = statePath[statePath.length - 1];
      FinishableState<T> fromState = initialState;

      List<StateTransitionAction> stateTransitionActionsCopy = new ArrayList<StateTransitionAction>(stateTransitionActions);

      for (FinishableState<T> toState : statePath)
      {
         StateTransitionCondition stateTransitionCondition = new RequestedStateTransitionCondition<T>(fromState, finalState.getStateEnum(), requestedState,
                                                                waitUntilDone);

         StateTransition<T> transition;
         if (toState == finalState)
         {
            StateTransitionAction stateTransitionAction = new StateTransitionAction()
            {
               public void doTransitionAction()
               {
                  requestedState.set(null);
               }
            };
            stateTransitionActionsCopy.add(stateTransitionAction);
         }

         transition = new StateTransition<T>(toState.getStateEnum(), stateTransitionCondition, stateTransitionActionsCopy);


         fromState.addStateTransition(transition);

         fromState = toState;
      }
   }

   private static class RequestedStateTransitionCondition<T extends Enum<T>> implements StateTransitionCondition
   {
      private final FinishableState<T> fromState;
      private final T requestedStateTrigger;
      private final boolean waitUntilDone;
      private final EnumYoVariable<T> requestedState;

      public RequestedStateTransitionCondition(FinishableState<T> fromState, T requestedStateTrigger, EnumYoVariable<T> requestedState, boolean waitUntilDone)
      {
         this.fromState = fromState;
         this.requestedStateTrigger = requestedStateTrigger;
         this.waitUntilDone = waitUntilDone;
         this.requestedState = requestedState;
      }

      public boolean checkCondition()
      {
         boolean done = !waitUntilDone || fromState.isDone();
         boolean transitionRequested = requestedStateTrigger.equals(requestedState.getEnumValue());

         return done && transitionRequested;
      }
   }

   private static <T> List<T> createListWithOneElement(T object)
   {
      ArrayList<T> ret = new ArrayList<>();
      ret.add(object);
      return ret;
   }
}
