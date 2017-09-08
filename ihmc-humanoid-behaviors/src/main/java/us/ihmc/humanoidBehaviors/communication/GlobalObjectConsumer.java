package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;

public class GlobalObjectConsumer implements GlobalPacketConsumer
{
   private final CommunicationBridgeInterface communicationBridgeInterface;
   public GlobalObjectConsumer(CommunicationBridgeInterface communicationBridgeInterface)
   {
      this.communicationBridgeInterface = communicationBridgeInterface;
   }

   @Override
   public void receivedPacket(Packet<?> packet)
   {
      communicationBridgeInterface.consumeObjectFromNetwork(packet);
   }
}
