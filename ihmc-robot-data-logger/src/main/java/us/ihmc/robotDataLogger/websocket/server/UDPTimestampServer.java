package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.PortUnreachableException;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import us.ihmc.log.LogTools;

/**
 * Thread that sends UDP messages with the timestamp to the clients to synchronize video data.
 * 
 * Sends in a seperate thread that polls for new timestamps every 0.1 ms.
 * Even if the NIO channel is non-blocking, it can result in a small and non-deterministic pause in the realtime thread, hence the need to run as a seperate thread. 
 * 
 * @author Jesper Smith
 *
 */
public class UDPTimestampServer
{
   public static final int TIMESTAMP_HEADER = 0x5d35bc23;
   
   private final Object closeLock = new Object();


   private final ByteBuffer sendDataBuffer = ByteBuffer.allocateDirect(12);
   private SocketAddress address;

   private final DatagramChannel channel;

   private volatile boolean active = false;


   public UDPTimestampServer() throws IOException
   {
      
      channel = DatagramChannel.open();
      channel.configureBlocking(false);
   }

   public void startSending(InetAddress target, int port)
   {
      this.address = new InetSocketAddress(target, port);
      try
      {
         this.channel.connect(address);
         this.active = true;
      }
      catch (IOException e)
      {
         LogTools.warn("Cannot connect UDP timestamp server to " + address + ": " + port + ". " + e.getMessage());
      }
   }

   public void sendTimestamp(long timestamp)
   {
      if(active)
      {
         sendDataBuffer.clear();
         sendDataBuffer.putInt(TIMESTAMP_HEADER);
         sendDataBuffer.putLong(timestamp);
         sendDataBuffer.flip();
         try
         {
            synchronized(closeLock)
            {
               if(active)
               {
                  channel.write(sendDataBuffer);
               }
            }
         }
         catch (PortUnreachableException e)
         {
            // Remote host disconnected
            active = false;
         }
         catch (IOException e)
         {
            LogTools.warn("Error sending timestamp. " + e.getMessage());
            active = false;
         }
      }

   }

   public void close()
   {
      synchronized(closeLock)
      {
         active = false;
         try
         {
            channel.close();
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot close UDP timestamp server. " + e.getMessage());
         }
      }
   }

}
