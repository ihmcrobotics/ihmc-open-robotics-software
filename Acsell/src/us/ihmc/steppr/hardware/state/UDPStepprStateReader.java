package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

public class UDPStepprStateReader extends RealtimeThread
{
   private final StepprState state;
   private final StepprStateProcessor stateProcessor;

   private final InetSocketAddress receiveAddress = new InetSocketAddress(11303);
   private DatagramChannel receiveChannel;
   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

   private volatile boolean requestStop = false;
   private boolean stateProcessorHasRun = false;

   
   public UDPStepprStateReader(PriorityParameters priority, StepprState state, StepprStateProcessor stateProcessor)
   {
      super(priority);
      this.state = state;
      this.stateProcessor = stateProcessor;
      
      receiveBuffer.order(ByteOrder.BIG_ENDIAN);
      
   }
   
   @Override
   public void run()
   {
      try
      {
         connect();
      }
      catch (IOException e)
      {
         disconnect();

         throw new RuntimeException(e);
      }

      System.gc();
      System.gc();
      
      try
      {

         while (!requestStop)
         {
            receiveBuffer.clear();
            receiveChannel.receive(receiveBuffer);
            receiveBuffer.flip();
            
            
            long currentTime = RealtimeThread.getCurrentMonotonicClockTime();
            if(state.update(receiveBuffer, currentTime))
            {
               if (!stateProcessorHasRun)
               {
                  stateProcessor.initialize(currentTime);
                  stateProcessorHasRun = true;
               }
               stateProcessor.process(currentTime);
            }                  
         }
      }
      catch (IOException e)
      {
         disconnect();

         throw new RuntimeException(e);
      }

      disconnect();
   }
   
   private void connect() throws IOException
   {
      receiveChannel = DatagramChannel.open();
      receiveChannel.socket().setReceiveBufferSize(65535);
      receiveChannel.socket().bind(receiveAddress);
   }

   private void disconnect()
   {
      try
      {
         receiveChannel.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

}
