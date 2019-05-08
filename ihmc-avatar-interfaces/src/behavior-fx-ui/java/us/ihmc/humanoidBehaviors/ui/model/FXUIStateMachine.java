package us.ihmc.humanoidBehaviors.ui.model;

import java.util.HashMap;

public class FXUIStateMachine
{
   private HashMap<FXUIStateTransitionTrigger, FXUIStateTransition> transitions = new HashMap<>();

   public FXUIStateMachine(FXUIStateTransition startTransition)
   {
      mapTransition(FXUIStateTransitionTrigger.START, startTransition);
   }

   public void mapTransition(FXUIStateTransitionTrigger trigger, FXUIStateTransition transition)
   {
      transitions.put(trigger, transition);
   }

   public final void transition(FXUIStateTransitionTrigger trigger)
   {
      transitions.get(trigger).transition(trigger);
   }

   public final void start()
   {
      transition(FXUIStateTransitionTrigger.START);
   }
}
