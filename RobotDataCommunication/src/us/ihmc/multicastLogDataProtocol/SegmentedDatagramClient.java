package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;

public class SegmentedDatagramClient extends Thread
{
   private static final int PACKAGE_BUFFER = 10;

   private final long sessionId;
   private final NetworkInterface iface;
   private final InetAddress group;
   private final LogPacketHandler handler;

   private final SegmentedPacketBuffer[] ringBuffer = new SegmentedPacketBuffer[PACKAGE_BUFFER];

   public SegmentedDatagramClient(long sessionId, NetworkInterface iface, InetAddress group, LogPacketHandler handler)
   {
      this.sessionId = sessionId;
      this.iface = iface;
      this.group = group;
      this.handler = handler;
   }

   private void checkComplete(int i)
   {
      if (ringBuffer[i].isComplete())
      {
         handler.newDataAvailable(ringBuffer[i]);
         ringBuffer[i] = null;
      }
   }

   private void updateBuffer(SegmentHeader header, ByteBuffer buffer)
   {
      int nextFree = -1;

      int oldest = -1;
      long oldestUid = Integer.MAX_VALUE;

      for (int i = PACKAGE_BUFFER - 1; i >= 0; --i)
      {

         if (ringBuffer[i] == null)
         {
            nextFree = i;
         }
         else
         {
            if (ringBuffer[i].getUid() == header.getPackageID())
            {
               ringBuffer[i].addSegment(header.getSegmentID(), buffer);
               checkComplete(i);
               return;
            }
            else
            {
               if (ringBuffer[i].getUid() < oldestUid)
               {
                  oldestUid = ringBuffer[i].getUid();
                  oldest = i;
               }
            }
         }
      }

      // First packet for this segmented package
      if (nextFree == -1)
      {
         nextFree = oldest;
      }

      ringBuffer[nextFree] = new SegmentedPacketBuffer(header);
      ringBuffer[nextFree].addSegment(header.getSegmentID(), buffer);
      handler.timestampReceived(header.getTimestamp());
      checkComplete(nextFree);
   }

   @Override
   public void run()
   {
      InetSocketAddress receiveAddress = new InetSocketAddress(LogDataProtocolSettings.LOG_DATA_PORT);
      SegmentHeader header = new SegmentHeader();

      ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

      DatagramChannel receiveChannel;
      MembershipKey receiveKey;
      try
      {
         receiveChannel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
               .bind(receiveAddress);
         receiveChannel.socket().setReceiveBufferSize(65535);
         receiveKey = receiveChannel.join(group, iface);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      while (!isInterrupted())
      {

         try
         {
            receiveBuffer.clear();
            receiveChannel.receive(receiveBuffer);
            receiveBuffer.flip();

            if(header.read(receiveBuffer))
            {
               updateBuffer(header, receiveBuffer);               
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

      }

      receiveKey.drop();
      try
      {
         receiveChannel.close();
      }
      catch (IOException e)
      {
      }

   }
}
