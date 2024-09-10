package us.ihmc.perception.streaming;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.UUID;

public class StreamingTools
{
   public static final UUID STATUS_MESSAGE_UUID = new UUID(0L, 0L);
   public static final double CONNECTION_TIMEOUT = 5.0; // 5 seconds to connect.

   public static String toSRTAddress(InetSocketAddress address)
   {
      return "srt://" + address.getHostString() + ":" + address.getPort();
   }

   public static InetAddress getHostIPAddress()
   {
      try
      {
         return NetworkInterface.networkInterfaces().filter(networkInterface ->
         {
            try
            {
               return networkInterface.isUp() && !networkInterface.isLoopback();
            }
            catch (SocketException e)
            {
               throw new RuntimeException(e);
            }
         }).map(networkInterface -> networkInterface.getInetAddresses().nextElement()).findAny().get();
      }
      catch (SocketException e)
      {
         throw new IllegalStateException("Could not find host IP address");
      }
   }
}
