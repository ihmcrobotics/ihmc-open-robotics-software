package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Random;

import us.ihmc.tools.thread.ThreadTools;

public class SegmentedPacketSimulation
{
   private static final String host = "127.0.0.1";   
   private final long sessionId = new Random().nextLong();
   
   
   public SegmentedPacketSimulation() throws IOException
   {
      createClient();
      createServer();
   }
   
   private ByteBuffer fillBuffer(int size)
   {
      ByteBuffer buffer = ByteBuffer.allocateDirect(size);
      Random random = new Random(1232);
      while(buffer.remaining() > 0)
      {
         buffer.put((byte) random.nextInt(255));
      }
      buffer.flip();
      return buffer;
   }
   
   private void createServer() throws IOException
   {
      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(host));
      InetAddress group = InetAddress.getByName("239.255.0.1");
      
      SegmentedDatagramServer server = new SegmentedDatagramServer(sessionId, iface, group, LogDataProtocolSettings.LOG_DATA_PORT_RANGE_START);
      
      long timestamp = 10;
      ByteBuffer buffer = fillBuffer(150000);
      while(true)
      {
         server.send(LogDataType.DATA, timestamp, buffer);
         timestamp++;
         ThreadTools.sleep(10);
      }

   }

   private void createClient() throws UnknownHostException, SocketException
   {
      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(host));
      InetAddress group = InetAddress.getByName("239.255.0.1");

      SegmentedDatagramClient client = new SegmentedDatagramClient(sessionId, iface, group, LogDataProtocolSettings.LOG_DATA_PORT_RANGE_START, new Handler());
      client.start();
   }
   
   
   private class Handler implements SegmentedLogPacketHandler 
   {
      private long lastUid;

      @Override
      public void timestampReceived(long timestamp)
      {
      }

      @Override
      public void newDataAvailable(SegmentedPacketBuffer buffer)
      {
         if(buffer.getUid() != lastUid +1)
         {
            System.err.println("MISSED PACKET");
         }
         lastUid = buffer.getUid();
         
         ByteBuffer res = buffer.getBuffer();
         Random random = new Random(1232);
         while(res.remaining() > 0)
         {
            if(res.get() != random.nextInt(255))
            {
               System.out.println("CORRUPTION");
               continue;
            }
         }
         
         System.out.println("HEY DATA " + buffer.getUid());
      }

      @Override
      public void timeout(long timeoutInMillis)
      {
         // TODO Auto-generated method stub
         
      }
      
   }
   
   public static void main(String[] args) throws IOException
   {
      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(host));
      
      new SegmentedPacketSimulation();
   }
}
