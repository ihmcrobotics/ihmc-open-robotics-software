package us.ihmc.avatar.colorVision;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;

import static us.ihmc.avatar.colorVision.DualBlackflyUDPSender.DATAGRAM_MAX_LENGTH;

public class DualBlackflyUDPReceiver
{
   private final SideDependentList<Thread> receiveThreads = new SideDependentList<>();
   private volatile boolean running;

   private final SideDependentList<byte[]> imageBuffers = new SideDependentList<>();
   private final SideDependentList<ImageDimensions> imageDimensions = new SideDependentList<>(ImageDimensions::new);

   public void start()
   {
      running = true;

      for (RobotSide side : RobotSide.values)
      {
         Thread receiveThread = new Thread(() ->
         {
            SocketAddress socketAddress = new InetSocketAddress(side == RobotSide.LEFT ? "192.1.0.1" : "192.1.0.1",
                                                                side == RobotSide.LEFT ?
                                                                      DualBlackflyUDPSender.LEFT_UDP_PORT :
                                                                      DualBlackflyUDPSender.RIGHT_UDP_PORT);
            DatagramSocket socket;

            try
            {
               socket = new DatagramSocket(socketAddress);
//               socket.setReceiveBufferSize(socket.getReceiveBufferSize() * 16);
            }
            catch (SocketException e)
            {
               LogTools.error("Unable to bind to address for blackfly UDP streaming: "  + socketAddress);
               return;
            }

            LogTools.info("Connecting to: " + socketAddress);

            byte[] buffer = new byte[(int) ((Math.pow(2, 16)) - 1)];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            while (running)
            {
               try
               {
                  socket.receive(packet);
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
               int maxFragmentDataLength = DATAGRAM_MAX_LENGTH - fragmentHeaderLength;
               int fragmentDataOffset = fragment * maxFragmentDataLength;

               byte[] imageBuffer = imageBuffers.get(sideFromDatagram);

               if (imageBuffer == null || imageBuffer.length != imageBytes)
               {
                  imageBuffer = new byte[imageBytes];
                  imageBuffers.put(sideFromDatagram, imageBuffer);
               }

               imageDimensions.get(sideFromDatagram).setImageWidth(width);
               imageDimensions.get(sideFromDatagram).setImageHeight(height);

               for (int i = 0; i < fragmentDataLength; i++)
               {
                  imageBuffer[fragmentDataOffset + i] = datagramBuffer.get(fragmentHeaderLength + i);
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

   public SideDependentList<byte[]> getImageBuffers()
   {
      return imageBuffers;
   }

   public SideDependentList<ImageDimensions> getImageDimensions()
   {
      return imageDimensions;
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

      dualBlackflyUDPReceiver.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPReceiver::stop));

      ThreadTools.sleepForever();
   }
}
