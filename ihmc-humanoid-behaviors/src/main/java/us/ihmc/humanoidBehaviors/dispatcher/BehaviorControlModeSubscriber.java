package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;

public class BehaviorControlModeSubscriber implements PacketConsumer<BehaviorControlModePacket>
{
   private final AtomicReference<BehaviorControlModePacket> packetReference = new AtomicReference<BehaviorControlModePacket>(null);

   public BehaviorControlModeSubscriber()
   {

   }

   public boolean checkForNewControlRequested()
   {
      return packetReference.get() != null;
   }

   public BehaviorControlModeEnum getRequestedBehaviorControl()
   {
      return BehaviorControlModeEnum.fromByte(packetReference.getAndSet(null).getBehaviorControlModeEnumRequest());
   }

   @Override
   public void receivedPacket(BehaviorControlModePacket object)
   {
      packetReference.set(object);
      System.out.println(getClass().getSimpleName() + " received packet");
   }
}
