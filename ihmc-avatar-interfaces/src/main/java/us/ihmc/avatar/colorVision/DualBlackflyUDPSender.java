package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;

public class DualBlackflyUDPSender
{
   private static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1);

   private final SpinnakerBlackflyManager spinnakerBlackflyManager = new SpinnakerBlackflyManager();
   private final SideDependentList<SpinnakerBlackfly> blackflyDevices = new SideDependentList<>();
   private final SideDependentList<SpinnakerBlackflyReadThread> blackflyReadThreads = new SideDependentList<>();
   private DatagramSocket socket;
   private InetAddress address;
   private final Thread publishThread;
   private volatile boolean running = true;

   public DualBlackflyUDPSender()
   {
      blackflyDevices.put(RobotSide.LEFT, spinnakerBlackflyManager.createSpinnakerBlackfly("22206798"));
      blackflyDevices.put(RobotSide.RIGHT, spinnakerBlackflyManager.createSpinnakerBlackfly("22206802"));

      blackflyReadThreads.put(RobotSide.LEFT, new SpinnakerBlackflyReadThread(RobotSide.LEFT, blackflyDevices.get(RobotSide.LEFT)));
      blackflyReadThreads.put(RobotSide.RIGHT, new SpinnakerBlackflyReadThread(RobotSide.RIGHT, blackflyDevices.get(RobotSide.RIGHT)));

      for (SpinnakerBlackflyReadThread blackflyReadThread : blackflyReadThreads)
      {
         blackflyReadThread.start();
      }

      try
      {
         socket = new DatagramSocket();
      }
      catch (SocketException e)
      {
         LogTools.error(e);
      }

      try
      {
         address = InetAddress.getByName("localhost");
      }
      catch (UnknownHostException e)
      {
         LogTools.error(e);
      }

      publishThread = new Thread(() ->
      {
         byte[] buffer = new byte[DATAGRAM_MAX_LENGTH];

         while (running)
         {
            String test = "test";

            buffer = test.getBytes();

            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, DualBlackflyUDPReceiver.BLACKFLY_UDP_PORT);

            try
            {
               socket.send(packet);
            }
            catch (IOException e)
            {
               LogTools.error(e);
            }

            ThreadTools.sleep(1000);
         }
      });

      publishThread.start();
   }

   public void stop()
   {
      running = false;

      try
      {
         publishThread.join();
      }
      catch (InterruptedException e)
      {
         LogTools.error(e);
      }

      for (SpinnakerBlackflyReadThread blackflyReadThread : blackflyReadThreads)
      {
         try
         {
            blackflyReadThread.join();
         }
         catch (InterruptedException e)
         {
            LogTools.error(e);
         }
      }

      spinnakerBlackflyManager.destroy();
   }

   public class SpinnakerBlackflyReadThread extends Thread
   {
      private final RobotSide side;
      private SpinnakerBlackfly spinnakerBlackfly;
      private final FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();

      public SpinnakerBlackflyReadThread(RobotSide side, SpinnakerBlackfly spinnakerBlackfly)
      {
         this.side = side;
         this.spinnakerBlackfly = spinnakerBlackfly;
      }

      @Override
      public void run()
      {
         while (running)
         {
            spinImage spinImage = new spinImage();

            spinnakerBlackfly.getNextImage(spinImage);

            frequencyStatisticPrinter.ping();

            int w = spinnakerBlackfly.getWidth(spinImage);
            int h = spinnakerBlackfly.getHeight(spinImage);
            int imageFrameSize = w * h;

            BytePointer spinImageData = new BytePointer(imageFrameSize); // close at the end
            spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

            int imageBytes = (int) spinImageData.limit(); // always
            int numberOfDatagramFragments = (int) Math.ceil((double) imageBytes / DATAGRAM_MAX_LENGTH);

            ByteBuffer byteBuffer = ByteBuffer.allocate(DATAGRAM_MAX_LENGTH);

            for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
            {
               int fragmentLength = Math.min(DATAGRAM_MAX_LENGTH, imageBytes - (fragment * DATAGRAM_MAX_LENGTH));

//               byteBuffer.putInt(fragmentLength);

                     // Write side
//               byteBuffer.put(side.toByte());


               byteBuffer.rewind();
            }

            // TODO: do something with spinImageData

            spinnakerBlackfly.releaseImage(spinImage);
            spinImageData.close();
         }

         spinnakerBlackfly.stopAcquiringImages();
         System.out.println("Stopped acquiring images");
      }
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPSender dualBlackflyUDPSender = new DualBlackflyUDPSender();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPSender::stop));

      ThreadTools.sleepForever();
   }
}
