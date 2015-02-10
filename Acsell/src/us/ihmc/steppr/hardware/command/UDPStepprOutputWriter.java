package us.ihmc.steppr.hardware.command;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;

public class UDPStepprOutputWriter
{
   private final StepprCommand command;
   
   private DatagramChannel channel;
   private InetSocketAddress streamAddress;
   private final ByteBuffer sendBuffer = ByteBuffer.allocate(65535);
   
   private int controlID;
   
   public UDPStepprOutputWriter(StepprCommand command)
   {
      this.command = command;
      sendBuffer.order(ByteOrder.LITTLE_ENDIAN);
     
   }
   
   public void connect()
   {
      try
      {
         NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(NetworkParameters.getHost(NetworkParameterKeys.robotController)));
         InetAddress group = InetAddress.getByName(StepprNetworkParameters.STEPPR_MULTICAST_GROUP);
         streamAddress = new InetSocketAddress(group, StepprNetworkParameters.UDP_MULTICAST_CONTROL_PORT);
         
        
         channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
               .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface);
         channel.join(group, iface);
   
         ByteBuffer command = ByteBuffer.allocate(5);
         command.put((byte) 1);
         command.put((byte) 0);
         command.put((byte) 1);
         command.put((byte) 1);
         command.put((byte) 0);
         command.flip();
         channel.send(command, streamAddress);
         
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      } 
   }
   
   public void write()
   {
      sendBuffer.clear();
      command.write(sendBuffer, controlID);
      sendBuffer.flip();
      try
      {
         channel.send(sendBuffer, streamAddress);
         controlID++;
      }
      catch (IOException e)
      {
         System.err.println(e.getMessage());
      }
   }
   
   public void disconnect()
   {
      try
      {
         channel.close();
      }
      catch (IOException e)
      {
         // Ignore
      }
      
   }
}
