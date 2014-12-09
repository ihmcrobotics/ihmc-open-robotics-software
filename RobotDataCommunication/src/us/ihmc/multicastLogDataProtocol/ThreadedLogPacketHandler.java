package us.ihmc.multicastLogDataProtocol;

import java.util.concurrent.ArrayBlockingQueue;

public class ThreadedLogPacketHandler extends Thread implements LogPacketHandler
{
   private final LogPacketHandler handler;
   private final ArrayBlockingQueue<SegmentedPacketBuffer> dataBuffer;
   
   private volatile boolean stopped;
   
   public ThreadedLogPacketHandler(LogPacketHandler handler, int capacity)
   {
      this.handler = handler;
      this.dataBuffer = new ArrayBlockingQueue<>(capacity);
   }
   
   @Override
   public void run()
   {
      while(!stopped)
      {
         try
         {
            SegmentedPacketBuffer buffer = dataBuffer.take();
            handler.newDataAvailable(buffer);
         }
         catch (InterruptedException e)
         {
         }
      }
      
      dataBuffer.clear();
   }
   
   public void shutdown()
   {
      stopped = true;
      interrupt();
   }
   

   @Override
   public void timestampReceived(long timestamp)
   {
      handler.timestampReceived(timestamp);
   }

   @Override
   public void newDataAvailable(SegmentedPacketBuffer buffer)
   {
      dataBuffer.offer(buffer);
   }

   @Override
   public void timeout(long timeoutInMillis)
   {
      handler.timeout(timeoutInMillis);
   }
   
}