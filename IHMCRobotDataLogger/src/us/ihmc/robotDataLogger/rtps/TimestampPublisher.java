package us.ihmc.robotDataLogger.rtps;

import java.util.concurrent.atomic.AtomicLong;

public class TimestampPublisher implements Runnable
{

   private final DataProducerParticipant participant;
   private final AtomicLong timestamp = new AtomicLong(Long.MIN_VALUE); 
   private long oldTimestamp = Long.MIN_VALUE;
   
   public TimestampPublisher(DataProducerParticipant participant)
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
