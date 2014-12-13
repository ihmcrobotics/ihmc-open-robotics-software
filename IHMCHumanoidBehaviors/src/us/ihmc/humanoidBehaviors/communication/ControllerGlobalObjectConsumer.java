package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;

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
