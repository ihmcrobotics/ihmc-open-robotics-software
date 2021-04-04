package us.ihmc.humanoidBehaviors.ui.model;

import java.util.HashMap;

public class FXUIActionMap
{
   private HashMap<FXUITrigger, FXUIAction> transitions = new HashMap<>();

   public FXUIActionMap(FXUIAction startAction)
   {
      mapAction(FXUITrigger.START, startAction);
   }

   public void mapAction(FXUITrigger trigger, FXUIAction action)
   {
      transitions.put(trigger, action);
   }

   public final void triggerAction(FXUITrigger trigger)
   {
      transitions.get(trigger).doAction(trigger);
   }

   public final void start()
   {
      triggerAction(FXUITrigger.START);
   }
}
