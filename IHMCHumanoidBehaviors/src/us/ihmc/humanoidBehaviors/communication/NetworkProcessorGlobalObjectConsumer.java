package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.utilities.net.GlobalObjectConsumer;

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
