package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;

public class NetworkProcessorGlobalObjectConsumer implements GlobalPacketConsumer
{
   private final BehaviorInterface behavior;
   public NetworkProcessorGlobalObjectConsumer(BehaviorInterface behavior)
   {
      this.behavior = behavior;
   }

   @Override
   public void receivedPacket(Packet<?> packet)
   {
      behavior.consumeObjectFromNetworkProcessor(packet);
   }
}
