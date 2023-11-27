package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;

public class DualBlackflyUDPSender
{
   public static final int BLACKFLY_LEFT_UDP_PORT = 9200;
   public static final int BLACKFLY_RIGHT_UDP_PORT = 9201;
   private static final int IPV4_HEADER_LENGTH = 28;
   private static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1) - IPV4_HEADER_LENGTH;

   private final DualBlackflyReader dualBlackflyReader = new DualBlackflyReader();
   private final SideDependentList<Thread> publishThreads = new SideDependentList<>();
   private volatile boolean running;

   public void start()
   {
      running = true;

      dualBlackflyReader.start();

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
               address = InetAddress.getByName(side == RobotSide.LEFT ? "192.1.0.1" : "192.2.0.1");
            }
            catch (UnknownHostException e)
            {
               LogTools.error(e);
               return;
            }

            Object notify = new Object();

            dualBlackflyReader.getSpinnakerBlackflyReaderThreads().get(side).setNotify(notify);

            FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();

            int frameNumber = 0;

            while (running)
            {
               synchronized (notify)
               {
                  try
                  {
                     notify.wait();
                  }
                  catch (InterruptedException e)
                  {
                     LogTools.error(e);
                  }
               }

               BytePointer imagePointer = dualBlackflyReader.getSpinnakerBlackflyReaderThreads().get(side).getLatestImageDataPointer().get();

               ByteBuffer datagramBuffer = ByteBuffer.allocate(DATAGRAM_MAX_LENGTH);

               int imageBytes = (int) imagePointer.limit();
               int numberOfDatagramFragments = (int) Math.ceil((double) imageBytes / DATAGRAM_MAX_LENGTH);

               for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
               {
                  int fragmentHeaderLength = 1 + 4 + 4 + 4 + 4 + 4 + 4;
                  int maxFragmentDataLength = DATAGRAM_MAX_LENGTH - fragmentHeaderLength;
                  int fragmentDataOffset = fragment * maxFragmentDataLength;
                  int fragmentDataLength = Math.min(maxFragmentDataLength, imageBytes - fragmentDataOffset);
                  int datagramLength = fragmentHeaderLength + fragmentDataLength;

                  // Sanity check
                  if (datagramLength > DATAGRAM_MAX_LENGTH)
                     throw new RuntimeException("Too many bytes");

                  // Write header data
                  {
                     datagramBuffer.put(side.toByte());
                     datagramBuffer.putInt(dualBlackflyReader.getSpinnakerBlackflyReaderThreads().get(side).getWidth());
                     datagramBuffer.putInt(dualBlackflyReader.getSpinnakerBlackflyReaderThreads().get(side).getHeight());
                     datagramBuffer.putInt(imageBytes);
                     datagramBuffer.putInt(frameNumber);
                     datagramBuffer.putInt(fragment);
                     datagramBuffer.putInt(fragmentDataLength);
                  }

                  // Write fragment data
                  {
                     ByteBuffer imageDataBuffer = imagePointer.asBuffer();
                     byte[] fragmentDataBytes = new byte[fragmentDataLength];

                     for (int i = 0; i < fragmentDataLength; i++)
                     {
                        fragmentDataBytes[i] = imageDataBuffer.get(fragmentDataOffset + i);
                     }

                     datagramBuffer.put(fragmentDataBytes);
                  }

                  byte[] datagramBytes = new byte[datagramLength];
                  datagramBuffer.rewind();
                  datagramBuffer.get(datagramBytes, 0, datagramLength);

                  DatagramPacket packet = new DatagramPacket(datagramBytes,
                                                             datagramLength,
                                                             address,
                                                             side == RobotSide.LEFT ? BLACKFLY_LEFT_UDP_PORT : BLACKFLY_RIGHT_UDP_PORT);
                  try
                  {
                     socket.send(packet);
                  }
                  catch (IOException e)
                  {
                     LogTools.error(e);
                  }

                  datagramBuffer.clear();
               }

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

      dualBlackflyReader.stop();
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPSender dualBlackflyUDPSender = new DualBlackflyUDPSender();

      dualBlackflyUDPSender.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPSender::stop));

      ThreadTools.sleepForever();
   }
}
