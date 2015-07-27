package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.AsynchronousCloseException;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;

import org.apache.commons.lang3.SystemUtils;

public class SegmentedDatagramClient extends Thread
{
   private static final int timeout = 1000;
   private static final int PACKAGE_BUFFER = 10;

   private final long sessionId;
   private final NetworkInterface iface;
   private final InetAddress group;
   private final SegmentedLogPacketHandler handler;
   private final int port;

   private final SegmentedPacketBuffer[] ringBuffer = new SegmentedPacketBuffer[PACKAGE_BUFFER];

   private volatile boolean running = true;
   
   public SegmentedDatagramClient(long sessionId, NetworkInterface iface, InetAddress group, int port, SegmentedLogPacketHandler handler)
   {
      super("SegmentedDataClient" + sessionId);
      this.sessionId = sessionId;
      this.iface = iface;
      this.group = group;
      this.handler = handler;
      this.port = port;
   }

   private boolean checkComplete(int i)
   {
      if (ringBuffer[i].isComplete())
      {
         handler.newDataAvailable(ringBuffer[i]);
         ringBuffer[i] = null;
         return true;
      }
      else
      {
         return false;
      }
   }
   
   public boolean isRunning()
   {
      return running;
   }

   private void updateBuffer(SegmentHeader header, ByteBuffer buffer)
   {
      int nextFree = -1;

      int oldest = -1;
      long oldestUid = Integer.MAX_VALUE;
      
      boolean newPacket = true;

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
               newPacket = false;
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

      if(newPacket)
      {
         // First packet for this segmented package
         if (nextFree == -1)
         {
            nextFree = oldest;
         }
      
         ringBuffer[nextFree] = new SegmentedPacketBuffer(header);
         ringBuffer[nextFree].addSegment(header.getSegmentID(), buffer);
         handler.timestampReceived(header.getTimestamp());
      }
      
      
      while(oldest != -1 && ringBuffer[oldest] != null && checkComplete(oldest))
      {
         oldest = (oldest + 1) % PACKAGE_BUFFER;
      }
   }

   @Override
   public void run()
   {
      InetSocketAddress receiveAddress;
      if(SystemUtils.IS_OS_WINDOWS)
      {
         // Windows doesn't allow binding to group IPs
         receiveAddress = new InetSocketAddress(port);
      }
      else
      {
         receiveAddress = new InetSocketAddress(group, port);
      }
      
      SegmentHeader header = new SegmentHeader();

      ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);

      DatagramChannel receiveChannel;
      MembershipKey receiveKey;
      Selector selector;
      SelectionKey key;
      try
      {
         System.out.println("Binding to " + receiveAddress + " on " + iface);
         receiveChannel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true).bind(receiveAddress);
         receiveChannel.configureBlocking(false);
         receiveKey = receiveChannel.join(group, iface);
         selector = Selector.open();
         key = receiveChannel.register(selector, SelectionKey.OP_READ);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      while (running)
      {

         try
         {
            if (selector.select(timeout) > 0)
            {
               selector.selectedKeys().remove(key);
               if (key.isReadable())
               {
                  receiveBuffer.clear();
                  receiveChannel.receive(receiveBuffer);
                  receiveBuffer.flip();

                  if (header.read(receiveBuffer))
                  {
                     if (header.getSessionID() == sessionId && header.getSegmentCount() > 0)
                     {
                        updateBuffer(header, receiveBuffer);
                     }
                  }
               }
            }
            else
            {
               handler.timeout(timeout);
            }
         }
         catch (AsynchronousCloseException e)
         {
            
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

   public void close()
   {
      running = false;

      if (Thread.currentThread() != this)
      {
         try
         {
            join();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }
}
