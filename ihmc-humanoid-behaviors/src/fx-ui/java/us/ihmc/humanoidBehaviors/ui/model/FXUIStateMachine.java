package us.ihmc.humanoidBehaviors.ui.model;

import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.messager.Messager;

import java.util.HashMap;

public abstract class FXUIStateMachine
{
   private final Messager messager;
   private final FXUIStateTransition exitTransition;
   private HashMap<FXUIStateTransition, FXUIState> stateMap = new HashMap<>();

   public FXUIStateMachine(Messager messager, FXUIState startState, FXUIStateTransition exitTransition)
   {
      this.messager = messager;
      this.exitTransition = exitTransition;

      mapTransitionToState(FXUIStateTransition.START, startState);
      mapTransitionToState(exitTransition, FXUIState.INACTIVE);
   }

   public final void start()
   {
      transition(FXUIStateTransition.START);
   }

   private final void exit()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, null);
   }

   public final void transition(FXUIStateTransition transition)
   {
      handleTransition(transition);

      if (transition.equals(exitTransition))
      {
         exit();
      }
   }

   protected abstract void handleTransition(FXUIStateTransition transition);

   public void mapTransitionToState(FXUIStateTransition transition, FXUIState state)
   {
      stateMap.put(transition, state);
   }
}
