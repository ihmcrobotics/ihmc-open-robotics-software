package us.ihmc.communication.packetCommunicator;

import java.util.ArrayList;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public abstract class PacketProducer<T extends Packet>
{
   private final ArrayList<PacketConsumer<T>> consumers = new ArrayList<PacketConsumer<T>>();
   
   public void addConsumer(PacketConsumer consumer)
   {
      consumers.add(consumer);
   }
   
   protected void sendObject(T object)
   {
      for(PacketConsumer<? super T> consumer : consumers)
      {
         consumer.receivedPacket(object);
      }
   }

}
