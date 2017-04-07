package us.ihmc.robotDataLogger.listeners;

public interface TimestampListener
{
   /**
    * Called when an UDP packet with the timestamp is received. The timestamps are send
    * over UDP for synchronization purposes. Timestamps are published directly from the realtime
    * thread and should have minimum delay
    * 
    * @param timestamp
    */
   void receivedTimestampOnly(long timestamp);
}
