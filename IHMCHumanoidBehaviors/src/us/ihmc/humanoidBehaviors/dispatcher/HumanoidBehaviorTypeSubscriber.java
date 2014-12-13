package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;

public class HumanoidBehaviorTypeSubscriber implements ObjectConsumer<HumanoidBehaviorTypePacket>
{
   private final AtomicReference<HumanoidBehaviorTypePacket> packetReference = new AtomicReference<HumanoidBehaviorTypePacket>(null);

   public HumanoidBehaviorTypeSubscriber()
   {
      
   }

   public boolean checkForNewBehaviorRequested()
   {
      return packetReference.get() != null;
   }

   public HumanoidBehaviorType getRequestedBehavior()
   {
      return packetReference.getAndSet(null).getBehaviorType();
   }

   @Override
   public void consumeObject(HumanoidBehaviorTypePacket object)
   {
      packetReference.set(object);
   }
}
