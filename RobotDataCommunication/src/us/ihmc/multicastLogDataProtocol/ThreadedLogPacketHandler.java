package us.ihmc.multicastLogDataProtocol;

import java.nio.ByteBuffer;
import java.util.concurrent.ArrayBlockingQueue;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotDataCommunication.LogDataHeader;

public class ThreadedLogPacketHandler extends Thread implements LogPacketHandler
{
   private final LogPacketHandler handler;
   private final ArrayBlockingQueue<ImmutablePair<LogDataHeader, ByteBuffer>> dataBuffer;
   
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
            ImmutablePair<LogDataHeader, ByteBuffer> buffer = dataBuffer.take();
            handler.newDataAvailable(buffer.getLeft(), buffer.getRight());
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
      // Immediately notify for video synchronization
      handler.timestampReceived(timestamp);
   }

   @Override
   public void newDataAvailable(LogDataHeader header, ByteBuffer buffer)
   {
      dataBuffer.offer(new ImmutablePair<>(header, buffer));
   }

   @Override
   public void timeout()
   {
      handler.timeout();
   }
   
}