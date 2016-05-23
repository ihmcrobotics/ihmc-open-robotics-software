package us.ihmc.humanoidBehaviors.communication;

import java.util.concurrent.ConcurrentLinkedQueue;

public class ConcurrentListeningQueue<T>
{
   private final ConcurrentLinkedQueue<T> packetQueue = new ConcurrentLinkedQueue<T>();
   private T lastPacket = null;

   public ConcurrentListeningQueue()
   {
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }
   
   public T getLatestPacket()
   {
      while (isNewPacketAvailable())
      {
         poll();
      }
      
      return lastPacket;
   }

   public T poll()
   {
      T polledPacket = packetQueue.poll();
      lastPacket = polledPacket;
      return polledPacket;
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
