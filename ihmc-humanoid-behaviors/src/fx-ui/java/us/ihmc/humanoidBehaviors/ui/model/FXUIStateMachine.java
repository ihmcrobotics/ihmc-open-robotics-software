package us.ihmc.humanoidBehaviors.ui.model;

import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.messager.Messager;

import java.util.HashMap;

public class FXUIStateMachine
{
   private final Messager messager;
   private final FXUIStateTransitionTrigger exitTransition;
   private HashMap<FXUIStateTransitionTrigger, FXUIStateTransition> transitions = new HashMap<>();

   public FXUIStateMachine(Messager messager, FXUIStateTransitionTrigger exitTrigger, FXUIStateTransition startTransition)
   {
      this.messager = messager;
      this.exitTransition = exitTrigger;

      mapTransition(FXUIStateTransitionTrigger.START, startTransition);
   }

   public void mapTransition(FXUIStateTransitionTrigger trigger, FXUIStateTransition transition)
   {
      transitions.put(trigger, transition);
   }

   public final void transition(FXUIStateTransitionTrigger trigger)
   {
      transitions.get(trigger).transition(trigger);

      if (trigger.equals(exitTransition))
      {
         deactivate();
      }
   }

   public final void start()
   {
      transition(FXUIStateTransitionTrigger.START);
   }

   private final void deactivate()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, null);
   }
}
