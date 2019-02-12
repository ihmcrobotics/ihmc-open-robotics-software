package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.concurrent.locks.LockSupport;

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

   private final UDPTimestampServerThread thread = new UDPTimestampServerThread();

   private final ByteBuffer sendDataBuffer = ByteBuffer.allocateDirect(12);
   private SocketAddress address;

   private volatile boolean active = false;

   private volatile long newTimestamp = Long.MIN_VALUE;

   public UDPTimestampServer() throws IOException
   {
   }

   public void startSending(InetAddress target, int port)
   {
      this.address = new InetSocketAddress(target, port); 
      this.active = true;
      this.thread.start();
   }

   public void sendTimestamp(long timestamp)
   {
      newTimestamp = timestamp;
   }

   public void close()
   {
      active = false;
   }

   /**
    * Internal thread that sends timestamps over a UDP connection.
    * 
    * Polls for new timestamps at approximately 10kHz
    * 
    * @author Jesper Smith
    *
    */
   private class UDPTimestampServerThread extends Thread
   {

      private UDPTimestampServerThread()
      {
         super(UDPTimestampServerThread.class.getSimpleName());
      }

      @Override
      public void run()
      {
         DatagramChannel channel;
         try
         {
            channel = DatagramChannel.open();
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot open UDP timestamp server. " + e.getMessage());
            active = false;
            return;
         }

         try
         {
            channel.configureBlocking(false);
            channel.connect(address);
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot connect to target. " + e.getMessage());
            active = false;
            try
            {
               channel.close();
            }
            catch (IOException ce)
            {
            }
            return;
         } 
         long lastSendTimestamp = Long.MIN_VALUE;

         while (active)
         {
            // Localize variable so it doesn't change in this thread
            long newTimestampLocal = newTimestamp;

            if (lastSendTimestamp != newTimestampLocal)
            {
               sendDataBuffer.clear();
               sendDataBuffer.putInt(TIMESTAMP_HEADER);
               sendDataBuffer.putLong(newTimestampLocal);
               sendDataBuffer.flip();
               try
               {
                  channel.write(sendDataBuffer);
                  lastSendTimestamp = newTimestampLocal;
               }
               catch (IOException e)
               {
                  LogTools.warn("Error sending timestamp. " + e.getMessage());
                  active = false;
               }               
            }
            
            LockSupport.parkNanos(100000); // 0.1 ms pause
         }

         try
         {
            channel.close();
         }
         catch (IOException e)
         {
         }

      }

   }
}
