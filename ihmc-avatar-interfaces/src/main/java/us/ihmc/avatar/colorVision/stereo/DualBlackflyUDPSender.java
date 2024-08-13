package us.ihmc.avatar.colorVision.stereo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.spinnaker.BlackflyModelProperties;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Properties;

public class DualBlackflyUDPSender
{
   private static final String LEFT_SERIAL_NUMBER = "17372478";
   private static final String RIGHT_SERIAL_NUMBER = "17403057";
   private static final BlackflyModelProperties BLACKFLY_MODEL = BlackflyModelProperties.BFLY_U3_23S6C;
   private static final String LEFT_DESTINATION_IP_ADDRESS = System.getProperty("blackfly.left.udp.dest.address", "127.0.0.1");
   private static final String RIGHT_DESTINATION_IP_ADDRESS = System.getProperty("blackfly.right.udp.dest.address", "127.0.0.1");
   public static final int LEFT_UDP_PORT = 9200;
   public static final int RIGHT_UDP_PORT = 9201;
   public static final int IPV4_HEADER_LENGTH = 28;
   public static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1) - IPV4_HEADER_LENGTH;

   private final SideDependentList<Thread> publishThreads = new SideDependentList<>();
   private volatile boolean running;

   private final File propertiesFile = IHMCCommonPaths.VR_DIRECTORY.resolve(getClass().getSimpleName() + ".properties").toFile();

   public void start()
   {
      running = true;

      SpinnakerBlackflyManager spinnakerBlackflyManager = new SpinnakerBlackflyManager();

      double originalWidth = BLACKFLY_MODEL.getImageWidthPixels();
      double originalHeight = BLACKFLY_MODEL.getImageHeightPixels();

      double croppedHeight = originalHeight;
      double xOffset = 0.0;
      double yOffset = 0.0;
      float exposureTimeUs = 6000.0f;

      if (propertiesFile.exists())
      {
         Properties properties = new Properties();

         try
         {
            properties.load(new FileInputStream(propertiesFile));
         }
         catch (IOException e)
         {
            LogTools.error(e);
         }

         try
         {
            croppedHeight = Integer.parseInt(properties.getProperty("croppedHeight"));
            xOffset = Double.parseDouble(properties.getProperty("xOffset"));
            yOffset = Double.parseDouble(properties.getProperty("yOffset"));
            exposureTimeUs = Float.parseFloat(properties.getProperty("exposureTimeUs"));
         }
         catch (NumberFormatException e)
         {
            LogTools.error(e);
         }
      }

      LogTools.info("originalWidth " + originalWidth);
      LogTools.info("originalHeight " + originalHeight);
      LogTools.info("croppedHeight " + croppedHeight);
      LogTools.info("exposureTimeUs " + exposureTimeUs);
      LogTools.info("xOffset " + xOffset);
      LogTools.info("yOffset " + yOffset);

      for (RobotSide side : RobotSide.values())
      {
         double finalCroppedHeight = croppedHeight;
         double finalXOffset = xOffset;
         double finalYOffset = yOffset;
         float finalExposureTimeUs = exposureTimeUs;
         Thread publishThread = new Thread(() ->
         {
            SpinnakerBlackfly spinnakerBlackfly = spinnakerBlackflyManager.createSpinnakerBlackfly(
                  side == RobotSide.LEFT ? LEFT_SERIAL_NUMBER : RIGHT_SERIAL_NUMBER,
                  (int) originalWidth,
                  (int) finalCroppedHeight,
                  (int) finalXOffset,
                  (int) finalYOffset,
                  finalExposureTimeUs);

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
               spinImage spinImage = new spinImage();

               spinnakerBlackfly.getNextImage(spinImage);

               int width = spinnakerBlackfly.getWidth(spinImage);
               int height = spinnakerBlackfly.getHeight(spinImage);
               int imageFrameSize = width * height;

               BytePointer spinImageData = new BytePointer(imageFrameSize);
               spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

               int latestImageDataLength = (int) spinImageData.limit();

               byte[] imageData = new byte[latestImageDataLength];
               spinImageData.get(imageData, 0, latestImageDataLength);

               int imageFrameSizeWithPadding = (int) (width * originalHeight);
               byte[] imageDataWithPadding = new byte[imageFrameSizeWithPadding];

               int paddingOffset = (imageFrameSizeWithPadding - imageFrameSize);

               for (int i = 0; i < imageDataWithPadding.length; i++)
               {
                  if (i < paddingOffset)
                  {
                     imageDataWithPadding[i] = 0;
                  }
                  else
                  {
                     imageDataWithPadding[i] = imageData[i - (paddingOffset)];
                  }
               }

               int numberOfDatagramFragments = (int) Math.ceil((double) imageFrameSizeWithPadding / DATAGRAM_MAX_LENGTH);

               for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
               {
                  int fragmentHeaderLength = 1 + 4 + 4 + 4 + 4 + 4 + 4;
                  int maxFragmentDataLength = DATAGRAM_MAX_LENGTH - fragmentHeaderLength;
                  int fragmentDataOffset = fragment * maxFragmentDataLength;
                  int fragmentDataLength = Math.min(maxFragmentDataLength, imageFrameSizeWithPadding - fragmentDataOffset);
                  int datagramLength = fragmentHeaderLength + fragmentDataLength;

                  byte[] datagramData = new byte[DATAGRAM_MAX_LENGTH];
                  ByteBuffer datagramBuffer = ByteBuffer.wrap(datagramData);

                  // Sanity check
                  if (datagramLength > DATAGRAM_MAX_LENGTH)
                     throw new RuntimeException("Too many bytes");

                  // Write header data
                  {
                     datagramBuffer.put(side.toByte());
                     datagramBuffer.putInt(width);
                     datagramBuffer.putInt(height);
                     datagramBuffer.putInt(imageFrameSizeWithPadding);
                     datagramBuffer.putInt(frameNumber);
                     datagramBuffer.putInt(fragment);
                     datagramBuffer.putInt(fragmentDataLength);
                  }

                  // Write fragment data
                  {
                     for (int i = 0; i < fragmentDataLength; i++)
                     {
                        datagramBuffer.put(imageDataWithPadding[fragmentDataOffset + i]);
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

               spinnakerBlackfly.releaseImage(spinImage);

               frameNumber++;

               frequencyStatisticPrinter.ping();
            }

            spinnakerBlackfly.stopAcquiringImages();

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
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPSender dualBlackflyUDPSender = new DualBlackflyUDPSender();

      dualBlackflyUDPSender.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPSender::stop));

      ThreadTools.sleepForever();
   }
}
