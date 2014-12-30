package us.ihmc.communication.packetCommunicator;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public class ConcurrentPacketQueue<T extends Packet> implements PacketConsumer<T>
{
   private final ConcurrentLinkedQueue<T> packetQueue = new ConcurrentLinkedQueue<T>();

   public ConcurrentPacketQueue()
   {
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }

   public T getPacket()
   {
      return packetQueue.poll();
   }
   
   public void put(T object)
   {
      packetQueue.add(object);
   }

   public void clear()
   {
      packetQueue.clear();
   }

   @Override
   public void receivedPacket(T packet)
   {
      packetQueue.add(packet);
   }
   
   public int size()
   {
      return packetQueue.size();
   }
}
