package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.robotDataLogger.dataBuffers.LoggerDebugRegistry;
import us.ihmc.robotDataLogger.dataBuffers.RegistryBuffer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBuffer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.util.PeriodicThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadScheduler;

public class RegistryPublisher
{
   private static final int BUFFER_CAPACITY = 128;

   private long uid = 0;
   private final ConcurrentRingBuffer<RegistrySendBuffer> ringBuffer;

   private final Publisher publisher;

   private final PeriodicThreadScheduler scheduler;
   private final VariableUpdateThread variableUpdateThread = new VariableUpdateThread();
   
   private final LoggerDebugRegistry loggerDebugRegistry;
   
   
   private final int[] segmentSizes;
   private final int[] segmentOffsets;


   public RegistryPublisher(PeriodicThreadSchedulerFactory schedulerFactory, RegistrySendBufferBuilder builder, Publisher publisher) throws IOException
   {
      this.segmentSizes = LogParticipantTools.calculateLogSegmentSizes(builder.getNumberOfVariables(), builder.getNumberOfJointStates());
      this.segmentOffsets = LogParticipantTools.calculateOffsets(this.segmentSizes);

      this.ringBuffer = new ConcurrentRingBuffer<>(builder, BUFFER_CAPACITY * segmentSizes.length);
      this.scheduler = schedulerFactory.createPeriodicThreadScheduler("Registry-" + builder.getRegistryID() + "-Publisher");
      this.publisher = publisher;
      
      this.loggerDebugRegistry = builder.getLoggerDebugRegistry();
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
      for(int segment = 0; segment < segmentSizes.length; segment++)
      {
         RegistrySendBuffer buffer = ringBuffer.next();
         if (buffer != null)
         {
            buffer.updateBufferFromVariables(timestamp, uid, segmentOffsets[segment], segmentSizes[segment]);
            ringBuffer.commit();
         }
         else
         {
            this.loggerDebugRegistry.circularBufferFull();
         }
         
         uid++;
         
      }
   }

   private class VariableUpdateThread implements Runnable
   {
      private long previousUid = -1;
      
      private VariableUpdateThread()
      {

      }

      @Override
      public void run()
      {
         try
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
                  
                  if(previousUid != -1)
                  {
                     if(buffer.getUid() != previousUid + 1)
                     {
                        loggerDebugRegistry.lostTickInCircularBuffer();
                     }
                  }
                  previousUid = buffer.getUid();
               }
               ringBuffer.flush();
            }

         }
         catch (Throwable e)
         {
            e.printStackTrace();
         }

      }

   }

}
