package us.ihmc.multicastLogDataProtocol;


public interface SegmentedLogPacketHandler
{
   /**
    * Gets called the first time a new timestamp is seen, even if the whole packet hasn't been received yet.
    * @param timestamp
    */
   public void timestampReceived(long timestamp);
   
   
   /**
    * Gets called when a new packet is available.
    * 
    * @param buffer
    */
   public void newDataAvailable(SegmentedPacketBuffer buffer);
   
   public void timeout(long timeoutInMillis);
}
