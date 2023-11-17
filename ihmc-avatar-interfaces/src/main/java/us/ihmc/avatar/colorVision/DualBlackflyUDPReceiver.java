package us.ihmc.avatar.colorVision;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class DualBlackflyUDPReceiver
{
   public static final int BLACKFLY_UDP_PORT = 7384;

   private DatagramSocket socket;
   private final Thread receiveThread;

   private volatile boolean running = true;

   public DualBlackflyUDPReceiver()
   {
      try
      {
         socket = new DatagramSocket(BLACKFLY_UDP_PORT);
      }
      catch (SocketException e)
      {
         LogTools.error(e);
      }

      System.out.println("Listening");

      receiveThread = new Thread(() ->
      {
         byte[] buffer = new byte[(int) ((Math.pow(2, 16)) - 1)];

         while (running)
         {
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            try
            {
               socket.receive(packet);
            }
            catch (IOException e)
            {
               LogTools.error(e);
            }

            System.out.println("recv");
         }
      });

      receiveThread.start();
   }

   public void stop()
   {
      running = false;

      try
      {
         receiveThread.join();
      }
      catch (InterruptedException e)
      {
         LogTools.error(e);
      }

      System.out.println("Stopped");
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPReceiver::stop));

      ThreadTools.sleepForever();
   }
}
