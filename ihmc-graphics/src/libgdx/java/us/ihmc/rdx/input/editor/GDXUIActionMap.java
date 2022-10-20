package us.ihmc.rdx.input.editor;

import java.util.HashMap;

public class GDXUIActionMap
{
   private HashMap<GDXUITrigger, GDXUIAction> transitions = new HashMap<>();

   public GDXUIActionMap(GDXUIAction startAction)
   {
      mapAction(GDXUITrigger.START, startAction);
   }

   public void mapAction(GDXUITrigger trigger, GDXUIAction action)
   {
      transitions.put(trigger, action);
   }

   public final void triggerAction(GDXUITrigger trigger)
   {
      transitions.get(trigger).doAction(trigger);
   }

   public final void start()
   {
      triggerAction(GDXUITrigger.START);
   }
}
