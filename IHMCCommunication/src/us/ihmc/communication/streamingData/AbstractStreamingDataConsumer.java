package us.ihmc.communication.streamingData;

import java.io.Serializable;


public abstract class AbstractStreamingDataConsumer<P extends Serializable> implements StreamingDataConsumer
{
   private final Class<P> packetType;
   private final long objectIdentifier;
   
   public AbstractStreamingDataConsumer(long objectIdentifier, Class<P> classToSearchFor)
   {
      this.objectIdentifier = objectIdentifier;
      packetType = classToSearchFor;
   }

   @SuppressWarnings("unchecked")
   final public void consume(long objectIdentifier, Object object)
   {
      if (this.objectIdentifier != objectIdentifier)
      {
         throw new RuntimeException("AbstractStreamingDataConsumer: Received objectIdentifier " + objectIdentifier + " but am expecting " + this.objectIdentifier);
      }
      if (canHandle(object))
      {
         processPacket((P) object);
      }
   }

   final public boolean canHandle(Object object)
   {
      return packetType.isInstance(object);
   }

   final public long getDataIdentifier()
   {
      return objectIdentifier;
   }
   
   protected abstract void processPacket(P packet);
}
