package us.ihmc.humanoidBehaviors.ui.model;

import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.messager.Messager;

import java.util.HashMap;

public abstract class FXUIStateMachine
{
   private final Messager messager;
   private final FXUIStateTransition exitTransition;
   private long lastStateTime = 0L;
   private FXUIState currentState = FXUIState.INACTIVE;
   private HashMap<FXUIStateTransition, FXUIState> stateMap = new HashMap<>();

   public FXUIStateMachine(Messager messager, FXUIState startState, FXUIStateTransition exitTransition)
   {
      this.messager = messager;
      this.exitTransition = exitTransition;

      mapTransitionToState(FXUIStateTransition.START, startState);
      mapTransitionToState(exitTransition, FXUIState.INACTIVE);
   }

   /**
    * Make sure changes don't activate until next tick.
    * @param now
    * @return current or INACTIVE if same tick as previous transition
    */
   public FXUIState currentState(long now)
   {
      if (now > lastStateTime)
      {
         return currentState;
      }
      else
      {
         return FXUIState.INACTIVE;
      }
   }

   public final void start()
   {
      transition(0L, FXUIStateTransition.START);
   }

   private final void exit()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, null);
   }

   public final void transition(long now, FXUIStateTransition transition)
   {
      lastStateTime = now;

      handleTransition(transition);

      currentState = stateMap.get(transition);

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
