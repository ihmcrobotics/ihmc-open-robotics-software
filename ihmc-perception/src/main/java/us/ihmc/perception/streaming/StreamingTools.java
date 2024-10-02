package us.ihmc.perception.streaming;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.SocketException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import static java.util.Map.entry;

public class StreamingTools
{
   public static final double CONNECTION_TIMEOUT = 2.0; // 2 seconds to connect.

   /**
    * For available options, see <a href="https://www.ffmpeg.org/ffmpeg-protocols.html#srt">FFMPEG srt documentation.</a>
    * To get a decent SRT configuration, see <a href="https://srtlab.github.io/srt-cookbook/protocol/configuration.html">SRT Configuration Calculator</a>
    */
   private static final Map<String, String> LIVE_SRT_OPTIONS
         = Map.ofEntries(entry("transtype", "live"),
                         entry("smoother", "live"),
                         entry("rcvlatency", "20"),
                         entry("peerlatency", "20"),     // 20ms of buffer delay for packet loss correction
                         entry("mss", "1360"),           // Max packet size of MPEG-TS
                         entry("payload_size", "1316")); // Payload size of MPEG-TS

   public static Map<String, String> getLiveSRTOptions()
   {
      return new HashMap<>(LIVE_SRT_OPTIONS);
   }

   /** hevc_nvenc options can be found using {@code ffmpeg -hide_banner -h encoder=hevc_nvenc}. */
   private static final Map<String, String> HEVC_NVENC_STREAMING_OPTIONS
         = Map.ofEntries(entry("preset", "p1"),       // p1 = fastest, p2 = fast, p3 = medium ... p7 = slowest
                         entry("tune", "ull"),        // "Ultra low latency"
                         entry("rc", "vbr"),          // Rate control: variable bit rate mode
                         entry("delay", "0"),         // No delay to frame output
                         entry("zerolatency", "1"));  // Don't introduce reordering delay

   public static Map<String, String> getHEVCNVENCStreamingOptions()
   {
      return new HashMap<>(HEVC_NVENC_STREAMING_OPTIONS);
   }

   /** FFV1 options can be found using {@code ffmpeg -hide_banner -h encoder=ffv1}. */
   private static final Map<String, String> FFV1_STREAMING_OPTIONS
         = Map.ofEntries(entry("coder", "range_def"),
                         entry("context", "0"),
                         entry("level", "3"),
                         entry("threads", "16"),
                         entry("slices", "24"),
                         entry("slicecrc", "0"),
                         entry("pass", "1"));

   public static Map<String, String> getFFV1StreamingOptions()
   {
      return new HashMap<>(FFV1_STREAMING_OPTIONS);
   }

   public static String toSRTAddress(InetSocketAddress address)
   {
      return "srt://" + address.getHostString() + ":" + address.getPort();
   }

   public static InetSocketAddress getHostAddress()
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
               return networkInterface.isUp() && !networkInterface.isLoopback();
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
