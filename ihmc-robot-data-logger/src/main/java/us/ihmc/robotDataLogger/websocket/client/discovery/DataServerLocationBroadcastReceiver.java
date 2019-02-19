package us.ihmc.robotDataLogger.websocket.client.discovery;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import io.netty.util.CharsetUtil;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.websocket.DataServerLocationBroadcast;

public class DataServerLocationBroadcastReceiver extends DataServerLocationBroadcast
{
   public interface DataServerLocationFoundListener
   {
      public void addHost(String host, int port, boolean persistant);
   }

   private final DataServerLocationFoundListener listener;
   private final ArrayList<Thread> threads = new ArrayList<>();

   private volatile boolean running;

   public DataServerLocationBroadcastReceiver(DataServerLocationFoundListener listener) throws IOException
   {
      List<MulticastSocket> sockets = getSocketChannelList(announcePort);
      for (MulticastSocket socket : sockets)
      {
         threads.add(new Thread(new DiscoveryEndpoint(socket)));
      }

      this.listener = listener;
   }

   public void start()
   {
      running = true;
      for (Thread thread : threads)
      {
         thread.start();
      }
   }

   public void stop()
   {
      running = false;
   }
   
   public void join()
   {
      for (Thread thread : threads)
      {
         try
         {
            thread.join();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   private class DiscoveryEndpoint implements Runnable
   {
      private final MulticastSocket socket;
      private final ObjectMapper mapper = new ObjectMapper(new JsonFactory());

      public DiscoveryEndpoint(MulticastSocket socket)
      {
         this.socket = socket;
      }

      @Override
      public void run()
      {
         try
         {
            InetAddress multicastAddress = InetAddress.getByName(announceGroupAddress);
            socket.joinGroup(multicastAddress);
            socket.setSoTimeout(1000);

            byte[] receiveBuffer = new byte[MAXIMUM_MESSAGE_SIZE];
            while (running)
            {
               DatagramPacket packet = new DatagramPacket(receiveBuffer, receiveBuffer.length);

               try
               {
                  socket.receive(packet);

                  String message = new String(packet.getData(), packet.getOffset(), packet.getLength(), CharsetUtil.UTF_8);

                  try
                  {
                     String host = packet.getAddress().getHostAddress();
                     int port = parseMessage(message, mapper);
                     listener.addHost(host, port, false);
                  }
                  catch (JsonParseException | JsonMappingException e)
                  {
                     LogTools.warn("Invalid message {}", e.getMessage());
                  }

               }
               catch (SocketTimeoutException e)
               {
               }
            }

            socket.leaveGroup(multicastAddress);
            socket.close();

         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

   }

   public static void main(String[] args) throws IOException
   {
      new DataServerLocationBroadcastReceiver(new DataServerLocationFoundListener()
      {
         
         @Override
         public void addHost(String host, int port, boolean persistant)
         {
            System.out.println("Found " + host +  ":" + port);
         }
      }).start();
   }
}
