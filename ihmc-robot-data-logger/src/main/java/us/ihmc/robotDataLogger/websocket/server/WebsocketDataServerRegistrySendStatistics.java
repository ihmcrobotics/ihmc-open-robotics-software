package us.ihmc.robotDataLogger.websocket.server;

/**
 * Helper to calculate loop rate of registries and determine if they should be send based on the requested rate 
 * 
 * @author Jesper Smith
 *
 */
class WebsocketDataServerRegistrySendStatistics
{
   private long registryLastTimestamp = 0;
   private long registryLastSendTimestamp = 0;
   private long registryDT = 0;
   private long jitter = 0;
   private long jitterSamples = 0;

   /**
    * Update estimated DT, jitter and last update timestamp
    * 
    * @param timestamp
    */
   public void update(long timestamp)
   {
      // Estimate registry ID using a simple alpha filter
      if(registryLastTimestamp != 0)
      {
         // Alpha filter the DT
         if(registryDT == 0) 
         {
            registryDT = (timestamp - registryLastTimestamp);
         }
         else
         {
            registryDT = (registryDT * 99 + (timestamp - registryLastTimestamp))/100;

            long D = (timestamp - registryLastTimestamp) - (registryDT);
            if(D < 0) D = -D;
            
            jitter += (D - jitter)/16;
            
            ++jitterSamples;
         }
         
      }
      
      
      registryLastTimestamp = timestamp;
   }
   
   /**
    * Test if the thread is non-monotonic.
    * 
    * Non-monotonic would be timestamp jitter more than the current registryDT/2
    * 
    * @return true if this thread is behaving non-monotonically
    */
   public boolean isNonMonotonic()
   {
      return registryDT == 0 || jitterSamples < 16 || jitter > (registryDT/2);
   }
   
   
   /** 
    * Figure out if we should send this registry based on a given timestamp
    * 
    * A registry should be send if the timestamp is (lastTimestamp + registryDT). To account for jitter, +- 0.5 * mainRegistryDT is the allowable range.
    * 
    * If checkForOtherRegistry is true, It checks if the requested timestamp is with +- 0.5 * mainRegistryDT of lastSendTimestamp + n * registryDT, where n can be any integer. 
    * 
    * @param timestamp Timestamp to check
    * @param requestedDT Reguested DT
    * @param fastestRegistryBufferDT The DT of registry 0
    * @param checkForOtherRegistry If checking to send another registry
    * 
    * @return
    */
   public boolean shouldSend(long timestamp, long requestedDT, long fastestRegistryBufferDT, boolean checkForOtherRegistry)
   {
      if(isNonMonotonic())
      {
         if(checkForOtherRegistry)
         {
            return false;
         }
         else
         {
            return timestamp > (registryLastSendTimestamp + requestedDT);
         }
      }

      long adjustedRequestedDT;
      
      if(requestedDT < registryDT)
      {
         adjustedRequestedDT = registryDT;
      }
      else
      {
         adjustedRequestedDT = Math.round(((double) requestedDT)/((double) registryDT)) * registryDT;
      }
      
      if(checkForOtherRegistry)
      {
                  
         long ticksAway = Math.round( ((double) (timestamp - registryLastSendTimestamp)) / ((double) adjustedRequestedDT) );
         long expectedSendTime = registryLastSendTimestamp + (ticksAway * adjustedRequestedDT);
         
         
         long minSendTimestamp = expectedSendTime - fastestRegistryBufferDT/2;
         long maxSendTimestamp = minSendTimestamp + fastestRegistryBufferDT;
         
         
         return (timestamp > minSendTimestamp && timestamp < maxSendTimestamp);
      }
      else
      {
         long minSendTimestamp = registryLastSendTimestamp + adjustedRequestedDT - (fastestRegistryBufferDT/2);
         return timestamp > minSendTimestamp;
      }
   }
   
   /**
    * Update when this registry has last send something
    * 
    * @param timestamp
    */
   public void updateSendTimestamp(long timestamp)
   {
      registryLastSendTimestamp = timestamp;
   }


   /**
    * Get the dt of this registry
    * 
    * @return
    */
   public long getRegistryBufferDT()
   {
      return registryDT;
   }
   
}