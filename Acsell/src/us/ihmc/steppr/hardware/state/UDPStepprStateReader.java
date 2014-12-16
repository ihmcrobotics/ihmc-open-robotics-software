package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketOption;
import java.net.SocketTimeoutException;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;

import us.ihmc.realtime.RealtimeThread;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;

public class UDPStepprStateReader
{
   private final StepprState state;

   private DatagramChannel receiveChannel;
   private MembershipKey receiveKey;
   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

   public UDPStepprStateReader(StepprState state)
   {
      this.state = state;
      receiveBuffer.order(ByteOrder.LITTLE_ENDIAN);
   }

   /**
    * Receive data from steppr
    * @return Current time or -1 if invalid
    * @throws IOException
    */
   public long receive() throws IOException
   {
      receiveBuffer.clear();
      try
      {
         receiveChannel.receive(receiveBuffer);
      }
      catch(SocketTimeoutException e)
      {
         throw new RuntimeException(e);
      }
      receiveBuffer.flip();

      if (receiveBuffer.remaining() != 1048)
      {
         throw new IOException("ReceiveBuffer is not the correct size");
      }

      long currentTime = RealtimeThread.getCurrentMonotonicClockTime();
      state.update(receiveBuffer, currentTime);
      return currentTime;
   }

   public void connect() throws IOException
   {

      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(StepprNetworkParameters.CONTROL_COMPUTER_HOST));
      System.out.println("Binding to interface: " + iface);

      InetSocketAddress receiveAddress = new InetSocketAddress(StepprNetworkParameters.UDP_MULTICAST_STATE_PORT);

      receiveChannel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(receiveAddress);
      receiveChannel.socket().setReceiveBufferSize(65535);
      receiveChannel.socket().setSoTimeout(1000);
      InetAddress group = InetAddress.getByName(StepprNetworkParameters.STEPPR_MULTICAST_GROUP);
      receiveKey = receiveChannel.join(group, iface);
   }

   public void disconnect()
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
