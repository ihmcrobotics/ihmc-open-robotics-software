package us.ihmc.robotDataLogger.rtps;

import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.robotDataLogger.interfaces.DataProducer;

@Deprecated
public class TimestampPublisher implements Runnable
{

   private final DataProducer participant;
   private final AtomicLong timestamp = new AtomicLong(Long.MIN_VALUE); 
   private long oldTimestamp = Long.MIN_VALUE;
   
   public TimestampPublisher(DataProducer participant)
   {
      this.participant = participant;
   }
   
   public void setTimestamp(long timestamp)
   {
      this.timestamp.set(timestamp);
   }
   
   @Override
   public void run()
   {
      long newTimestamp = timestamp.get();
      
      if(oldTimestamp < newTimestamp)
      {
         participant.publishTimestamp(newTimestamp);
         oldTimestamp = newTimestamp;
      }
   }  
   

}
