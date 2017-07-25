package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.robotDataLogger.dataBuffers.KeepAliveBuffer;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public class KeepAlivePublisher
{
   
   private final Publisher publisher;
   private final KeepAliveBuffer buffer;
   private final PeriodicThreadScheduler scheduler; 
   private final KeepAlivePublisherRunner runner = new KeepAlivePublisherRunner();
   
   KeepAlivePublisher(PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory, Publisher publisher)
   {
      this.publisher = publisher;
      this.buffer = new KeepAliveBuffer();
      this.scheduler = periodicThreadSchedulerFactory.createPeriodicThreadScheduler("KeepAlivePublisher");
   }
   
   public void start()
   {
      scheduler.schedule(runner, 100, TimeUnit.MILLISECONDS);
   }
   
   public void stop()
   {
      scheduler.shutdown();
      try
      {
         scheduler.awaitTermination(1, TimeUnit.SECONDS);
      }
      catch (InterruptedException e)
      {
      }
   }
    
   private class KeepAlivePublisherRunner implements Runnable
   {
      
      private long uid = 0;

      @Override
      public void run()
      {
         try
         {
            buffer.setUid(uid);
            buffer.setTransmitTime(System.nanoTime());
            publisher.write(buffer);
            uid++;
         }
         catch (IOException e)
         {
            if(publisher.isAvailable())
            {
               e.printStackTrace();
            }
         }
      }
      
   }
   
}
