package us.ihmc.multicastLogDataProtocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Random;

public class SegmentedPacketSimulation
{
   private static final String host = "192.168.29.243";

   private final long sessionId = new Random().nextLong();
   
   
   public SegmentedPacketSimulation() throws IOException
   {
      createClient();
      createServer();
   }
   
   private ByteBuffer fillBuffer(int size)
   {
      ByteBuffer buffer = ByteBuffer.allocateDirect(size);
      Random random = new Random();
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
      
      SegmentedDatagramServer server = new SegmentedDatagramServer(sessionId, iface, group);
      
      long timestamp = 10;
      ByteBuffer buffer = fillBuffer(15000);
      while(true)
      {
         server.send(LogDataType.DATA, timestamp, buffer);
         timestamp++;
      }

   }

   private void createClient() throws UnknownHostException, SocketException
   {
      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(host));
      InetAddress group = InetAddress.getByName("239.255.0.1");

      SegmentedDatagramClient client = new SegmentedDatagramClient(sessionId, iface, group, new Handler());
      client.start();
   }
   
   
   private class Handler implements LogPacketHandler 
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
         System.out.println("HEY DATA " + buffer.getUid());
      }
      
   }
   
   public static void main(String[] args) throws IOException
   {
      NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(host));
      System.out.println("MTU: " + iface.getMTU());
      
      new SegmentedPacketSimulation();
   }
}
