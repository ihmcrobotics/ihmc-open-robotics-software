package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import us.ihmc.log.LogTools;

public class UDPTimestampServer
{
   public static final int TIMESTAMP_HEADER = 0x5d35bc23;
   
   private final Object lock = new Object();
   
   private final DatagramChannel channel;
   
   private final ByteBuffer sendDataBuffer = ByteBuffer.allocateDirect(12);
   
   private SocketAddress address;
   private boolean active = false;
   
   public UDPTimestampServer() throws IOException
   {
      channel = DatagramChannel.open();
      channel.configureBlocking(false);
   }
   
   public void startSending(InetAddress target, int port)
   {
      synchronized (lock)
      {
         this.address = new InetSocketAddress(target, port);
         try
         {
            channel.connect(address);
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot start UDP timestamp server: " + e.getMessage());
         }
         active = true;         
      }
   }
   
   
   public void sendTimestamp(long timestamp)
   {
      synchronized(lock)
      {
         if(active)
         {
            sendDataBuffer.clear();
            sendDataBuffer.putInt(TIMESTAMP_HEADER);
            sendDataBuffer.putLong(timestamp);
            sendDataBuffer.flip();
            
            
            try
            {
               channel.write(sendDataBuffer);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      }
   }
   
   public void close() 
   {
      synchronized(lock)
      {
         active = false;
         try
         {
            channel.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }
}
