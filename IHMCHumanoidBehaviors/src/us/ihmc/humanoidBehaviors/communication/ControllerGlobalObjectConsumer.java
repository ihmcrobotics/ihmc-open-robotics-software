package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.utilities.net.GlobalObjectConsumer;

public class ControllerGlobalObjectConsumer implements GlobalObjectConsumer
{
   private final BehaviorInterface behavior;
   public ControllerGlobalObjectConsumer(BehaviorInterface behavior)
   {
      this.behavior = behavior;
   }

   @Override
   public void consumeObject(Object object)
   {
      behavior.consumeObjectFromController(object);
   }

   @Override
   public void consumeObject(Object object, boolean consumeGlobal)
   {
      behavior.consumeObjectFromController(object);
   }
}
