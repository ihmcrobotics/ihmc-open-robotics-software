package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;

public class HumanoidBehaviorTypeSubscriber implements PacketConsumer<HumanoidBehaviorTypePacket>
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
   public void receivedPacket(HumanoidBehaviorTypePacket object)
   {
      packetReference.set(object);
      System.out.println(getClass().getSimpleName() + " received packet");
   }
}
