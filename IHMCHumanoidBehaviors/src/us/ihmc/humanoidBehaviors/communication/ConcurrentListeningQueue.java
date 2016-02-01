package us.ihmc.humanoidBehaviors.communication;

import java.util.concurrent.ConcurrentLinkedQueue;

public class ConcurrentListeningQueue<T>
{

   private final ConcurrentLinkedQueue<T> packetQueue = new ConcurrentLinkedQueue<T>();

   public ConcurrentListeningQueue()
   {
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }

   public T getNewestPacket()
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
}

