package us.ihmc.communication.remote;

import java.io.Serializable;

import us.ihmc.communication.streamingData.AbstractStreamingDataConsumer;

public class StreamingDataConsumerForTesting<T extends Serializable> extends AbstractStreamingDataConsumer<T>
{
   private volatile long packetReceivedIndex = 0;    // volatile for safety, not 100% sure it's necessary
   private T lastReceivedPacket = null;

   public StreamingDataConsumerForTesting(long objectIdentifier, Class<T> classToSearchFor)
   {
      super(objectIdentifier,  classToSearchFor);
   }

   public long getLastPacketReceivedIndex()
   {
      return packetReceivedIndex;
   }

   protected synchronized void processPacket(T packet)
   {
      packetReceivedIndex++;
      this.lastReceivedPacket = packet;
      notifyAll();
   }

   public T getLastReceivedPacket()
   {
      return lastReceivedPacket;
   } 

   
}
