package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;

public class HumanoidBehaviorControlModeSubscriber implements PacketConsumer<HumanoidBehaviorControlModePacket>
{
   private final AtomicReference<HumanoidBehaviorControlModePacket> packetReference = new AtomicReference<HumanoidBehaviorControlModePacket>(null);

   public HumanoidBehaviorControlModeSubscriber()
   {

   }

   public boolean checkForNewControlRequested()
   {
      return packetReference.get() != null;
   }

   public HumanoidBehaviorControlModeEnum getRequestedBehaviorControl()
   {
      return packetReference.getAndSet(null).getRequestedControl();
   }

   @Override
   public void receivedPacket(HumanoidBehaviorControlModePacket object)
   {
      packetReference.set(object);
      System.out.println(getClass().getSimpleName() + " received packet");
   }
}
