package us.ihmc.avatar.colorVision.stereo;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyCalculator;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;

import static us.ihmc.avatar.colorVision.stereo.DualBlackflyUDPSender.DATAGRAM_MAX_LENGTH;

public class DualBlackflyUDPReceiver
{
   private final SideDependentList<Thread> receiveThreads = new SideDependentList<>();
   private volatile boolean running = false;
   private volatile boolean connected = false;

   private final SideDependentList<String> addresses = new SideDependentList<>();
   private final SideDependentList<byte[]> imageBuffers = new SideDependentList<>();
   private final SideDependentList<ImageDimensions> imageDimensions = new SideDependentList<>(ImageDimensions::new);
   private final SideDependentList<FrequencyCalculator> frequencyCalculators;

   public DualBlackflyUDPReceiver()
   {
      frequencyCalculators = new SideDependentList<>();
      frequencyCalculators.put(RobotSide.LEFT, new FrequencyCalculator(10));
      frequencyCalculators.put(RobotSide.RIGHT, new FrequencyCalculator(10));

      addresses.put(RobotSide.LEFT, "172.16.66.230");
      addresses.put(RobotSide.RIGHT, "172.16.66.230");
   }

   public boolean connected()
   {
      return connected;
   }

   public void start()
   {
      running = true;

      for (RobotSide side : RobotSide.values)
      {
         Thread receiveThread = new Thread(() ->
                                           {
                                              SocketAddress socketAddress = new InetSocketAddress(addresses.get(side),
                                                                                                  side == RobotSide.LEFT ?
                                                                                                        DualBlackflyUDPSender.LEFT_UDP_PORT :
                                                                                                        DualBlackflyUDPSender.RIGHT_UDP_PORT);
                                              DatagramSocket socket;

                                              try
                                              {
                                                 socket = new DatagramSocket(socketAddress);
                                              }
                                              catch (SocketException e)
                                              {
                                                 e.printStackTrace();
                                                 LogTools.error("Unable to bind to address for Blackfly UDP streaming: " + socketAddress);
                                                 return;
                                              }

                                              LogTools.info("Connecting to: " + socketAddress);

                                              byte[] buffer = new byte[(int) ((Math.pow(2, 16)) - 1)];
                                              DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

                                              while (running)
                                              {
                                                 try
                                                 {
                                                    socket.setSoTimeout(1000);
                                                    socket.receive(packet);
                                                    connected = true;
                                                 }
                                                 catch (IOException e)
                                                 {
                                                    LogTools.error(e);
                                                    connected = false;
                                                    running = false;
                                                    break;
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

                                                 frequencyCalculators.get(side).ping();
                                              }

                                              socket.disconnect();
                                              socket.close();

                                              connected = false;
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
         if (receiveThread == null)
         {
            continue;
         }

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

   public void setAddress(RobotSide side, String address)
   {
      addresses.put(side, address);
   }

   public SideDependentList<String> getAddresses()
   {
      return addresses;
   }

   public SideDependentList<byte[]> getImageBuffers()
   {
      return imageBuffers;
   }

   public SideDependentList<ImageDimensions> getImageDimensions()
   {
      return imageDimensions;
   }

   public SideDependentList<FrequencyCalculator> getFrequencyCalculators()
   {
      return frequencyCalculators;
   }

   public static void main(String[] args)
   {
      DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

      dualBlackflyUDPReceiver.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyUDPReceiver::stop));

      ThreadTools.sleepForever();
   }
}
