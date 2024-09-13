package us.ihmc.perception.streaming;

import us.ihmc.commons.Conversions;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.SocketException;
import java.util.Optional;
import java.util.UUID;

public class StreamingTools
{
   public static final UUID STATUS_MESSAGE_UUID = new UUID(0L, 0L);
   public static final double CONNECTION_TIMEOUT = 2.0; // 2 seconds to connect.

   public static String toSRTAddress(InetSocketAddress address)
   {
      return "srt://" + address.getHostString() + ":" + address.getPort();
   }

   public static InetSocketAddress getMyAddress()
   {
      return new InetSocketAddress(getHostIPAddress(), getOpenPort());
   }

   public static int getOpenPort()
   {
      try (ServerSocket tempSocket = new ServerSocket(0))
      {
         return tempSocket.getLocalPort();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static InetAddress getHostIPAddress()
   {
      try
      {
         Optional<InetAddress> myIpv4Address = NetworkInterface.networkInterfaces()
         .filter(networkInterface ->
         {
            try
            {
               return networkInterface.isUp() && ! networkInterface.isLoopback();
            }
            catch (SocketException exception)
            {
               throw new RuntimeException(exception);
            }
         })
         .flatMap(NetworkInterface::inetAddresses)
         .filter(address -> address instanceof Inet4Address).findAny();
         return myIpv4Address.orElse(null);
      }
      catch (SocketException e)
      {
         throw new RuntimeException(e);
      }
   }
}
