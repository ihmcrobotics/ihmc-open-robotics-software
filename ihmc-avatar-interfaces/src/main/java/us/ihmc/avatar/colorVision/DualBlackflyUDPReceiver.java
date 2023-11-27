package us.ihmc.avatar.colorVision;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;

public class DualBlackflyUDPReceiver
{
   private final SideDependentList<Thread> receiveThreads = new SideDependentList<>();
   private DualBlackflyFragmentedUDPImage latestCompleteLeftImage;
   private DualBlackflyFragmentedUDPImage latestCompleteRightImage;
   private volatile boolean running;

   public void start()
   {
      running = true;

      for (RobotSide side : RobotSide.values())
      {
         Thread receiveThread = new Thread(() ->
         {
            FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();

            SocketAddress socketAddress = new InetSocketAddress(side == RobotSide.LEFT ? "192.1.0.1" : "192.2.0.1",
                                                                side == RobotSide.LEFT ?
                                                                      DualBlackflyUDPSender.BLACKFLY_LEFT_UDP_PORT :
                                                                      DualBlackflyUDPSender.BLACKFLY_RIGHT_UDP_PORT);
            DatagramSocket socket;

            try
            {
               socket = new DatagramSocket(socketAddress);
            }
            catch (SocketException e)
            {
               LogTools.error(e);
               return;
            }

            System.out.println(socketAddress);

            byte[] buffer = new byte[(int) ((Math.pow(2, 16)) - 1)];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            DualBlackflyFragmentedUDPImage fragmentedUDPImage = null;

            System.out.println(socket.isBound());

            while (running)
            {
               try
               {
                  socket.receive(packet);

                  frequencyStatisticPrinter.ping();
               }
               catch (IOException e)
               {
                  LogTools.error(e);
               }

               ByteBuffer datagramBuffer = ByteBuffer.wrap(buffer);
               int fragmentHeaderLength = 1 + 4 + 4 + 4 + 4 + 4 + 4;
               RobotSide sideFromDatagram = RobotSide.fromByte(datagramBuffer.get());
               int width = datagramBuffer.getInt();
               int height = datagramBuffer.getInt();
               int imageBytes = datagramBuffer.getInt();
               int frameNumber = datagramBuffer.getInt();
               int fragment = datagramBuffer.getInt();
               int fragmentDataLength = datagramBuffer.getInt();
               byte[] fragmentDataBytes = new byte[fragmentDataLength];

               for (int i = 0; i < fragmentDataLength; i++)
               { // TODO: +1 ?
                  fragmentDataBytes[i] = datagramBuffer.get(fragmentHeaderLength + i);
               }

               if (fragment == 0)
               {
                  // Should only happen once
                  if (fragmentedUDPImage == null)
                  {
                     fragmentedUDPImage = new DualBlackflyFragmentedUDPImage(imageBytes);
                  }
                  else
                  {
                     fragmentedUDPImage.complete();
                     System.out.println("complete");

                     switch (sideFromDatagram)
                     {
                        case LEFT -> latestCompleteLeftImage = fragmentedUDPImage.clone();
                        case RIGHT -> latestCompleteRightImage = fragmentedUDPImage.clone();
                     }

                     fragmentedUDPImage.reset();
                     fragmentedUDPImage.setRobotSide(sideFromDatagram);
                     fragmentedUDPImage.setWidth(width);
                     fragmentedUDPImage.setHeight(height);
                     fragmentedUDPImage.setFrameNumber(frameNumber);
                  }
               }
               else
               {
                  if (fragmentedUDPImage != null)
                     fragmentedUDPImage.nextFragment(fragment, fragmentDataLength, fragmentDataBytes);
               }
            }

            socket.disconnect();
            socket.close();
         });

         receiveThreads.put(side, receiveThread);

         receiveThread.start();
      }
   }

   public void stop()
   {
      running = false;

      for (Thread receiveThread : receiveThreads)
      {
         try
         {
            receiveThread.join();
         }
         catch (InterruptedException e)
         {
            LogTools.error(e);
         }
      }
   }

   public DualBlackflyFragmentedUDPImage getLatestCompleteLeftImage()
   {
      return latestCompleteLeftImage;
   }

   public DualBlackflyFragmentedUDPImage getLatestCompleteRightImage()
   {
      return latestCompleteRightImage;
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

      dualBlackflyUDPReceiver.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPReceiver::stop));

      ThreadTools.sleepForever();
   }
}
