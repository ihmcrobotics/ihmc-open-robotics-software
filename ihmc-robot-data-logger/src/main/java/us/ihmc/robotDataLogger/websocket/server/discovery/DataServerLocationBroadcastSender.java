package us.ihmc.robotDataLogger.websocket.server.discovery;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.util.List;

import io.netty.util.CharsetUtil;
import us.ihmc.robotDataLogger.websocket.DataServerLocationBroadcast;

/**
 * Simple tool to broadcast all local IP addresses on multicast address "announceGroupAddress" every second.
 */
public class DataServerLocationBroadcastSender extends DataServerLocationBroadcast
{
   private volatile boolean running = true;

   private final byte[] message;

   private final Thread internalThread;

   public DataServerLocationBroadcastSender(int dataServerPort) throws IOException
   {
      String messageStr = createMessage(dataServerPort);
      message = messageStr.getBytes(CharsetUtil.UTF_8);

      if (message.length > MAXIMUM_MESSAGE_SIZE)
      {
         throw new IOException("Message to big.");
      }

      this.internalThread = new Thread(() -> run());
   }

   public void start()
   {
      running = true;
      internalThread.start();
   }

   public void stop()
   {
      running = false;
      try
      {
         internalThread.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private void run()
   {
      try
      {
         List<MulticastSocket> sockets = getSocketChannelList(0);
         InetAddress group = InetAddress.getByName(announceGroupAddress);

         DatagramPacket packet = new DatagramPacket(message, message.length, group, announcePort);

         while (running)
         {
            for (int i = 0; i < sockets.size(); i++)
            {
               sockets.get(i).send(packet);
            }

            try
            {
               Thread.sleep(1000);
            }
            catch (InterruptedException e)
            {
            }
         }

         for (int i = 0; i < sockets.size(); i++)
         {
            sockets.get(i).close();
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args) throws IOException
   {
      new DataServerLocationBroadcastSender(8008).start();
   }
}
