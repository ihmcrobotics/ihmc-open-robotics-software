package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;

public class NetworkProcessorGlobalObjectConsumer implements GlobalObjectConsumer
{
   private final BehaviorInterface behavior;
   public NetworkProcessorGlobalObjectConsumer(BehaviorInterface behavior)
   {
      this.behavior = behavior;
   }

   @Override
   public void consumeObject(Object object)
   {
      behavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   public void consumeObject(Object object, boolean consumeGlobal)
   {
      behavior.consumeObjectFromNetworkProcessor(object);
   }
}
