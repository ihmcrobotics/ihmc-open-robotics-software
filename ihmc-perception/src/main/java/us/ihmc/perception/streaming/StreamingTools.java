package us.ihmc.perception.streaming;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;

public class StreamingTools
{
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
