package us.ihmc.communication.streamingData;

public interface StreamingDataProducer
{
   public abstract void registerConsumer(StreamingDataConsumer consumer);
   public abstract long getDataIdentifier();
}
