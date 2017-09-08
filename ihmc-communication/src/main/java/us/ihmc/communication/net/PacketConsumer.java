package us.ihmc.communication.net;

import us.ihmc.communication.packets.Packet;

public interface PacketConsumer<T extends Packet>
{
   public abstract void receivedPacket(T packet);
}
