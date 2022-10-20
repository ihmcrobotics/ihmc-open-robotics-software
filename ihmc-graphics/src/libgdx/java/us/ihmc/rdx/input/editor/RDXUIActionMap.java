package us.ihmc.rdx.input.editor;

import java.util.HashMap;

public class RDXUIActionMap
{
   private HashMap<RDXUITrigger, RDXUIAction> transitions = new HashMap<>();

   public RDXUIActionMap(RDXUIAction startAction)
   {
      mapAction(RDXUITrigger.START, startAction);
   }

   public void mapAction(RDXUITrigger trigger, RDXUIAction action)
   {
      transitions.put(trigger, action);
   }

   public final void triggerAction(RDXUITrigger trigger)
   {
      transitions.get(trigger).doAction(trigger);
   }

   public final void start()
   {
      triggerAction(RDXUITrigger.START);
   }
}
