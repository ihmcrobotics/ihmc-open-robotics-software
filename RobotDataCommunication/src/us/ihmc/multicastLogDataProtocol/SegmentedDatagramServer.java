package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

public class SegmentedDatagramServer
{   
   private final int maximumPacketSize;
   private final int payloadSize;

   private final DatagramChannel channel;
   private final InetSocketAddress address;

   private final SegmentHeader header = new SegmentHeader();
   private final ByteBuffer sendBuffer;

   private final long sessionID;
   private volatile long uid = 0;
   /**
    * Constructor for multicast server
    * @throws IOException 
    */
   public SegmentedDatagramServer(long sessionID, NetworkInterface iface, InetAddress group, int port) throws IOException
   {
      this.sessionID = sessionID;
      System.out.println("Binding Segmented Datagram Server to " + iface);
      if(iface.isLoopback())
      {
         maximumPacketSize = 65536 - 32;
      }
      else
      {
         maximumPacketSize = iface.getMTU() - 32; // IP header(20 bytes) + UDP Header (8 bytes) + wiggle room (4 bytes)
      }
      payloadSize = maximumPacketSize - SegmentHeader.HEADER_SIZE;
      address = new InetSocketAddress(group, port);

      channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
            .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface);

      sendBuffer = ByteBuffer.allocate(maximumPacketSize);
   }

   public void send(LogDataType type, long timestamp, ByteBuffer data) throws IOException
   {
      int segmentCount = data.remaining() / payloadSize + 1;

      header.setType(type);
      header.setSessionID(sessionID);
      header.setPackageID(++uid);
      header.setSegmentCount(segmentCount);
      header.setTimestamp(timestamp);
      
      for (int segment = 0; segment < segmentCount; segment++)
      {
         sendBuffer.clear();
         header.setSegmentID(segment);
         header.write(sendBuffer);

         int bytesToTransfer = Math.min(data.remaining(), payloadSize);

         for (int i = 0; i < bytesToTransfer; i++)
         {
            sendBuffer.put(data.get());
         }
         sendBuffer.flip();
         channel.send(sendBuffer, address);
      }
   }

   public void close()
   {
      try
      {
         channel.close();
      }
      catch (IOException e)
      {
      }
   }
}
