package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;

public class NetworkProcessorGlobalObjectConsumer implements GlobalPacketConsumer
{
   private final AbstractBehavior behavior;
   public NetworkProcessorGlobalObjectConsumer(AbstractBehavior behavior)
   {
      this.behavior = behavior;
   }

   @Override
   public void receivedPacket(Packet<?> packet)
   {
      behavior.consumeObjectFromNetworkProcessor(packet);
   }
}
