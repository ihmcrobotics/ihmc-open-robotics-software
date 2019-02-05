package us.ihmc.robotDataLogger.websocket.client;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;

import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.websocket.server.UDPTimestampServer;

/**
 * Simple client for the UDP timestamps
 * 
 * @author Jesper Smith
 *
 */
public class UDPTimestampClient
{
   private final DatagramSocket clientSocket;
   private final int port;

   private final Thread thread;
   private volatile boolean running = true;
   
   private final TimestampListener listener; 

   public UDPTimestampClient(TimestampListener listener) throws SocketException
   {
      this.listener = listener;
      
      clientSocket = new DatagramSocket();
      clientSocket.setSoTimeout(100);
      port = clientSocket.getLocalPort();

      thread = new Thread(new ReceiveThread());
   }

   public void start()
   {
      if (running && !thread.isAlive())
      {
         thread.start();
      }
      else
      {
         throw new RuntimeException("Thread already started");
      }
   }

   public int getPort()
   {
      return port;
   }

   public void stop()
   {
      running = false;
   }

   public void join()
   {
      try
      {
         thread.join();
      }
      catch (InterruptedException e)
      {
      }

   }

   private class ReceiveThread implements Runnable
   {

      @Override
      public void run()
      {
         byte[] data = new byte[12];
         ByteBuffer wrappedData = ByteBuffer.wrap(data);
         DatagramPacket incoming = new DatagramPacket(data, 12);
         while (running)
         {
            try
            {
               clientSocket.receive(incoming);
               if(wrappedData.getInt(0) == UDPTimestampServer.TIMESTAMP_HEADER)
               {
                  listener.receivedTimestampOnly(wrappedData.getLong(4));
               }
            }
            catch (SocketTimeoutException e)
            {
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }
      }
   }
}
