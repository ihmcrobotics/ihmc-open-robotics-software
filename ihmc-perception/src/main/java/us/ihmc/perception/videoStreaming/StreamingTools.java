package us.ihmc.perception.videoStreaming;

import java.net.InetSocketAddress;

public class StreamingTools
{
   public static String toSRTAddress(InetSocketAddress address)
   {
      return "srt://" + address.getHostString() + ":" + address.getPort();
   }
}
