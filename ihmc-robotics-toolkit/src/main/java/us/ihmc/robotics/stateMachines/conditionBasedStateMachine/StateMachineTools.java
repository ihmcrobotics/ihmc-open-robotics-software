package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.variable.YoEnum;

/**
 * @author twan
 *         Date: 5/30/13
 */
public class StateMachineTools
{
   @SafeVarargs
   public static <T extends Enum<T>> void addRequestedStateTransition(final YoEnum<T> requestedState, boolean waitUntilDone,
           final FinishableState<T> initialState, final FinishableState<T>... statePath)
   {
      addRequestedStateTransition(requestedState, waitUntilDone, new ArrayList<StateTransitionAction>(), initialState, statePath);
   }

   @SafeVarargs
   public static <T extends Enum<T>> void addRequestedStateTransition(final YoEnum<T> requestedState, boolean waitUntilDone,
         StateTransitionAction stateTransitionAction, final FinishableState<T> initialState, final FinishableState<T>... statePath)
   {
      addRequestedStateTransition(requestedState, waitUntilDone, createListWithOneElement(stateTransitionAction), initialState, statePath);
   }

   @SafeVarargs
   public static <T extends Enum<T>> void addRequestedStateTransition(final YoEnum<T> requestedState, boolean waitUntilDone,
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

   public static <T extends Enum<T>> StateTransition<T> buildRequestableStateTransition(final YoEnum<T> requestedState, final T finalStateEnum)
   {
      return buildRequestableStateTransition(requestedState, new ArrayList<>(), finalStateEnum);
   }

   public static <T extends Enum<T>> StateTransition<T> buildRequestableStateTransition(final YoEnum<T> requestedState,
                                                                                      List<? extends StateTransitionAction> stateTransitionActions,
                                                                                      final T finalStateEnum)
   {
      List<StateTransitionAction> stateTransitionActionsCopy = new ArrayList<>(stateTransitionActions);
      StateTransitionCondition stateTransitionCondition = new RequestedStateTransitionCondition<T>(finalStateEnum, requestedState);
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            requestedState.set(null);
         }
      };
      stateTransitionActionsCopy.add(stateTransitionAction);

      return new StateTransition<T>(finalStateEnum, stateTransitionCondition, stateTransitionActionsCopy);
   }

   public static <T extends Enum<T>> StateTransition<T> buildFinishedStateTransition(final FinishableState<T> initialState, final T finalStateEnum)
   {
      return buildFinishedStateTransition(initialState, new ArrayList<>(), finalStateEnum);
   }

   public static <T extends Enum<T>> StateTransition<T> buildFinishedStateTransition(final FinishableState<T> initialState,
                                                                                     List<? extends StateTransitionAction> stateTransitionActions,
                                                                                     final T finalStateEnum)
   {
      List<StateTransitionAction> stateTransitionActionsCopy = new ArrayList<>(stateTransitionActions);
      StateTransitionCondition stateTransitionCondition = new FinishedStateTransitionCondition<>(initialState, finalStateEnum);
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
         }
      };
      stateTransitionActionsCopy.add(stateTransitionAction);

      return new StateTransition<T>(finalStateEnum, stateTransitionCondition, stateTransitionActionsCopy);
   }

   private static class RequestedStateTransitionCondition<T extends Enum<T>> implements StateTransitionCondition
   {
      private final FinishableState<T> fromState;
      private final T requestedStateTrigger;
      private final boolean waitUntilDone;
      private final YoEnum<T> requestedState;

      public RequestedStateTransitionCondition(T requestedStateTrigger, YoEnum<T> requestedState)
      {
         this(null, requestedStateTrigger, requestedState, false);
      }

      public RequestedStateTransitionCondition(FinishableState<T> fromState, T requestedStateTrigger, YoEnum<T> requestedState, boolean waitUntilDone)
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

   private static class FinishedStateTransitionCondition<T extends Enum<T>> implements StateTransitionCondition
   {
      private final FinishableState<T> fromState;
      private final T nextState;

      public FinishedStateTransitionCondition(FinishableState<T> fromState, T nextState)
      {
         this.fromState = fromState;
         this.nextState = nextState;
      }

      public boolean checkCondition()
      {
         return fromState.isDone();
      }
   }

   private static <T> List<T> createListWithOneElement(T object)
   {
      ArrayList<T> ret = new ArrayList<>();
      ret.add(object);
      return ret;
   }
}
