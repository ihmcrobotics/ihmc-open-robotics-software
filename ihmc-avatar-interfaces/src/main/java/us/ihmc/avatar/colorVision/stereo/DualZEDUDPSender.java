package us.ihmc.avatar.colorVision.stereo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;

public class DualZEDUDPSender
{
   private static final String LEFT_DESTINATION_IP_ADDRESS = System.getProperty("zed.left.udp.dest.address", "127.0.0.1");
   private static final String RIGHT_DESTINATION_IP_ADDRESS = System.getProperty("zed.right.udp.dest.address", "127.0.0.1");
   public static final int LEFT_UDP_PORT = 9200;
   public static final int RIGHT_UDP_PORT = 9201;
   public static final int IPV4_HEADER_LENGTH = 28;
   public static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1) - IPV4_HEADER_LENGTH;

   private final SideDependentList<Thread> publishThreads = new SideDependentList<>();
   private volatile boolean running;

   private ZEDColorDepthImageRetriever zedColorDepthImageRetriever;

   private ReferenceFrame sensorFrame = ReferenceFrame.getWorldFrame();

   private ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }

   public void start()
   {
      running = true;

      zedColorDepthImageRetriever = new ZEDColorDepthImageRetriever(0, this::getSensorFrame, null, null);

      zedColorDepthImageRetriever.start();

      for (RobotSide side : RobotSide.values())
      {
         Thread publishThread = new Thread(() ->
         {
            DatagramSocket socket;
            try
            {
               socket = new DatagramSocket();
            }
            catch (SocketException e)
            {
               LogTools.error(e);
               return;
            }
            InetAddress address;
            try
            {
               address = InetAddress.getByName(side == RobotSide.LEFT ? LEFT_DESTINATION_IP_ADDRESS : RIGHT_DESTINATION_IP_ADDRESS);
            }
            catch (UnknownHostException e)
            {
               LogTools.error(e);
               return;
            }

            FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();

            int frameNumber = 0;

            while (running)
            {
               RawImage image = zedColorDepthImageRetriever.getLatestRawColorImage(side);
               Mat cpuMat = image.getCpuImageMat();

               BytePointer dataPointer = cpuMat.ptr();

               int latestImageDataLength = (int) (cpuMat.total() * cpuMat.elemSize());

               byte[] imageData = new byte[latestImageDataLength];
               dataPointer.get(imageData, 0, latestImageDataLength);

               int numberOfDatagramFragments = (int) Math.ceil((double) cpuMat.limit() / DATAGRAM_MAX_LENGTH);

               for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
               {
                  int fragmentHeaderLength = 1 + 4 + 4 + 4 + 4 + 4 + 4;
                  int maxFragmentDataLength = DATAGRAM_MAX_LENGTH - fragmentHeaderLength;
                  int fragmentDataOffset = fragment * maxFragmentDataLength;
                  int fragmentDataLength = Math.min(maxFragmentDataLength, latestImageDataLength - fragmentDataOffset);
                  int datagramLength = fragmentHeaderLength + fragmentDataLength;

                  byte[] datagramData = new byte[DATAGRAM_MAX_LENGTH];
                  ByteBuffer datagramBuffer = ByteBuffer.wrap(datagramData);

                  // Sanity check
                  if (datagramLength > DATAGRAM_MAX_LENGTH)
                     throw new RuntimeException("Too many bytes");

                  // Write header data
                  {
                     datagramBuffer.put(side.toByte());
                     datagramBuffer.putInt(image.getImageWidth());
                     datagramBuffer.putInt(image.getImageHeight());
                     datagramBuffer.putInt(latestImageDataLength);
                     datagramBuffer.putInt(frameNumber);
                     datagramBuffer.putInt(fragment);
                     datagramBuffer.putInt(fragmentDataLength);
                  }

                  // Write fragment data
                  {
                     for (int i = 0; i < fragmentDataLength; i++)
                     {
                        datagramBuffer.put(imageData[fragmentDataOffset + i]);
                     }
                  }

                  DatagramPacket packet = new DatagramPacket(datagramData, datagramLength, address, side == RobotSide.LEFT ? LEFT_UDP_PORT : RIGHT_UDP_PORT);
                  try
                  {
                     socket.send(packet);
                  }
                  catch (IOException e)
                  {
                     LogTools.error(e);
                  }
               }

               image.release();

               frameNumber++;

               frequencyStatisticPrinter.ping();
            }

            socket.close();
         });

         publishThreads.put(side, publishThread);

         publishThread.start();
      }
   }

   public void stop()
   {
      running = false;

      for (Thread publishThread : publishThreads)
      {
         try
         {
            publishThread.join();
         }
         catch (InterruptedException e)
         {
            LogTools.error(e);
         }
      }

      zedColorDepthImageRetriever.stop();
   }

   public static void main(String[] args)
   {
      DualZEDUDPSender dualZEDUDPSender = new DualZEDUDPSender();
      dualZEDUDPSender.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualZEDUDPSender::stop));

      ThreadTools.sleepForever();
   }
}
