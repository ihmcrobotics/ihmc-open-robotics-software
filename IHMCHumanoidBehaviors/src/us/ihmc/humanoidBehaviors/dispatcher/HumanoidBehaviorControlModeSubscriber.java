package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;

public class HumanoidBehaviorControlModeSubscriber implements ObjectConsumer<HumanoidBehaviorControlModePacket>
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
   public void consumeObject(HumanoidBehaviorControlModePacket object)
   {
      packetReference.set(object);
   }
}
