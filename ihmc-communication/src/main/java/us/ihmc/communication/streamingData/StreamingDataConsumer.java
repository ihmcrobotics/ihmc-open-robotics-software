package us.ihmc.communication.streamingData;


public interface StreamingDataConsumer
{
   public abstract boolean canHandle(Object object);
   public abstract void consume(long dataIdentifier, Object dataObject);
   public abstract long getDataIdentifier();

}
