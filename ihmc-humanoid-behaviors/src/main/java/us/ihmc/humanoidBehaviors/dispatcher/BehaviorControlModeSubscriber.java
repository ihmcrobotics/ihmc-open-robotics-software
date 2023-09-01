package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.concurrent.atomic.AtomicReference;

import toolbox_msgs.msg.dds.BehaviorControlModePacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.log.LogTools;

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
      LogTools.info("Received {}. Source: {}",
                    BehaviorControlModeEnum.fromByte(object.getBehaviorControlModeEnumRequest()).name(),
                    object.getSource());
   }
}
