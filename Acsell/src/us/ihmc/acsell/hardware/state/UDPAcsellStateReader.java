package us.ihmc.acsell.hardware.state;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketTimeoutException;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;

import us.ihmc.acsell.hardware.configuration.AcsellNetworkParameters;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.realtime.RealtimeThread;

public class UDPAcsellStateReader
{
   private final AcsellState<?,?> state;

   private DatagramChannel receiveChannel;
   private MembershipKey receiveKey;
   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

   public UDPAcsellStateReader(AcsellState<?,?> state)
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
      catch (SocketTimeoutException e)
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

      NetworkInterface iface = LogUtils.getMyInterface(NetworkParameters.getHost(NetworkParameterKeys.robotController));
      System.out.println("Binding to interface: " + iface);

      InetSocketAddress receiveAddress = new InetSocketAddress(AcsellNetworkParameters.UDP_MULTICAST_STATE_PORT);

      receiveChannel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(receiveAddress);
      receiveChannel.socket().setReceiveBufferSize(65535);
      receiveChannel.socket().setSoTimeout(1000);
      InetAddress group = InetAddress.getByName(AcsellNetworkParameters.ACSELL_MULTICAST_GROUP);
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
