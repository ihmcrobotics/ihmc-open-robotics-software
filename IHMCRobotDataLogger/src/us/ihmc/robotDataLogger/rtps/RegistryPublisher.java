package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.robotDataLogger.dataBuffers.RegistryBuffer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBuffer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.util.PeriodicThreadScheduler;

public class RegistryPublisher
{
   private static final int BUFFER_CAPACITY = 128;
   
   private long uid = 0;
   private final ConcurrentRingBuffer<RegistrySendBuffer> ringBuffer;
   
   private final Publisher publisher;
   
   private final PeriodicThreadScheduler scheduler;
   private final VariableUpdateThread variableUpdateThread = new VariableUpdateThread();
   
   public RegistryPublisher(PeriodicThreadScheduler scheduler, RegistrySendBufferBuilder builder, Publisher publisher) throws IOException
   {
      this.ringBuffer = new ConcurrentRingBuffer<>(builder, BUFFER_CAPACITY);
      
      this.publisher = publisher;
      this.scheduler = scheduler;
   }
   
   public void start()
   {
      scheduler.schedule(variableUpdateThread, 1, TimeUnit.MILLISECONDS);
   }
   
   public void stop()
   {
      scheduler.shutdown();
   }
   
   public void update(long timestamp)
   {
      RegistrySendBuffer buffer = ringBuffer.next();
      if(buffer != null)
      {
         buffer.updateBufferFromVariables(timestamp, uid);
         ringBuffer.commit();
      }
      
      uid++;
   }
   
   
   private class VariableUpdateThread implements Runnable
   {
      private VariableUpdateThread()
      {
         
      }

      @Override
      public void run()
      {
         while (ringBuffer.poll())
         {
            RegistryBuffer buffer;

            if ((buffer = ringBuffer.read()) != null)
            {
               
               
               try
               {
                  publisher.write(buffer);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
            ringBuffer.flush();
         }
            
      }
      
      
   }
   
}
