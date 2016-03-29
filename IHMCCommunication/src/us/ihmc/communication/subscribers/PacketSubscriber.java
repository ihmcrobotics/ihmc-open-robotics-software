package us.ihmc.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

public class PacketSubscriber<T>
{
	   private final ConcurrentLinkedQueue<T> packetQueue = new ConcurrentLinkedQueue<>();
	   
	   public void consumeObject(T newestPacket)
	   {
	      packetQueue.add(newestPacket);
	   }
	   
	   public boolean hasNewPacket()
	   {
	      return !packetQueue.isEmpty();
	   }

	   public T getPacket()
	   {
	      return packetQueue.poll();
	   }
}
