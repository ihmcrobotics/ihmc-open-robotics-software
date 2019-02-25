package us.ihmc.humanoidBehaviors.ui.behaviors;

import java.util.ArrayDeque;
import java.util.HashMap;

public abstract class FXUIStateMachine
{
   public static final FXUIStateMachine NONE = new FXUIStateMachine()
   {
      @Override
      protected void handleTransition(FXUIStateTransition transition)
      {
         // empty
      }
   };

   private final ArrayDeque<Object> deque = new ArrayDeque<>();

   private long lastStateTime = 0L;

   private FXUIState currentState = FXUIState.INACTIVE;

   private HashMap<FXUIStateTransition, FXUIState> stateMap = new HashMap<>();

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

   public final void begin()
   {
      transition(0L, FXUIStateTransition.BEGIN);
   }

   public final void transition(long now, FXUIStateTransition transition)
   {
      lastStateTime = now;

      handleTransition(transition);

      currentState = stateMap.get(transition);
   }

   protected abstract void handleTransition(FXUIStateTransition transition);

   public void mapTransitionToState(FXUIStateTransition transition, FXUIState state)
   {
      stateMap.put(transition, state);
   }
}
