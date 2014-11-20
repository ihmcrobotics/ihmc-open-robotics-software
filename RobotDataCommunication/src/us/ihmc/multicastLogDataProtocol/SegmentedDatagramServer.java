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
   public SegmentedDatagramServer(long sessionID, NetworkInterface iface, InetAddress group) throws IOException
   {
      this.sessionID = sessionID;
      maximumPacketSize = iface.getMTU() - 28; // IP header(20 bytes) + UDP Header (8 bytes)
      payloadSize = maximumPacketSize - SegmentHeader.HEADER_SIZE;
      address = new InetSocketAddress(group, LogDataProtocolSettings.LOG_DATA_PORT);

      channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
            .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface);
//      channel.join(group, iface);

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
}
