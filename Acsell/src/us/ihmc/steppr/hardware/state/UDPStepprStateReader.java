package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;

public class UDPStepprStateReader extends RealtimeThread
{
   private final StepprState state;
   private final StepprStateProcessor stateProcessor;

   private DatagramChannel receiveChannel;
   private MembershipKey receiveKey; 
   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

   private volatile boolean requestStop = false;
   private boolean stateProcessorHasRun = false;

   
   public UDPStepprStateReader(PriorityParameters priority, StepprState state, StepprStateProcessor stateProcessor)
   {
      super(priority);
      this.state = state;
      this.stateProcessor = stateProcessor;
      
      receiveBuffer.order(ByteOrder.LITTLE_ENDIAN);
      
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
            
            if(receiveBuffer.remaining() != 1048)
            {
               continue;
            }
            
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
      
      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(StepprNetworkParameters.CONTROL_COMPUTER_HOST));
      System.out.println("Binding to interface: " + iface);
      
      InetSocketAddress receiveAddress = new InetSocketAddress(StepprNetworkParameters.UDP_MULTICAST_STATE_PORT);
      
      receiveChannel = DatagramChannel.open(StandardProtocolFamily.INET)
            .setOption(StandardSocketOptions.SO_REUSEADDR, true)
            .bind(receiveAddress);
      receiveChannel.socket().setReceiveBufferSize(65535);
      
      InetAddress group = InetAddress.getByName(StepprNetworkParameters.STEPPR_MULTICAST_GROUP);
      receiveKey = receiveChannel.join(group, iface);
   }

   private void disconnect()
   {
      try
      {
         receiveKey.drop();
         receiveChannel.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

}
