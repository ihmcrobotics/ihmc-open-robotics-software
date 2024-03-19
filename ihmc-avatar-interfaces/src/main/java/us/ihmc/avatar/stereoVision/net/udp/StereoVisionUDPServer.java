package us.ihmc.avatar.stereoVision.net.udp;

import us.ihmc.avatar.stereoVision.net.packet.StereoVisionImageFragmentPacket;
import us.ihmc.avatar.stereoVision.net.packet.StereoVisionPacketListener;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyCalculator;

import javax.annotation.Nullable;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.util.concurrent.atomic.AtomicInteger;

// UI process
public class StereoVisionUDPServer implements StereoVisionPacketListener
{
   private final SideDependentList<byte[]> imageBuffers = new SideDependentList<>();
   private final SideDependentList<ImageDimensions> imageDimensions = new SideDependentList<>(ImageDimensions::new);
   private final SideDependentList<AtomicInteger> frameNumbers = new SideDependentList<>();
   private final SideDependentList<FrequencyCalculator> frequencyCalculators = new SideDependentList<>();
   @Nullable
   private InetSocketAddress bindAddress;
   @Nullable
   private DatagramSocket socket;
   @Nullable
   private Thread receiveThread;

   private volatile boolean running = false;

   public StereoVisionUDPServer()
   {
      frameNumbers.put(RobotSide.LEFT, new AtomicInteger(0));
      frameNumbers.put(RobotSide.RIGHT, new AtomicInteger(0));

      frequencyCalculators.put(RobotSide.LEFT, new FrequencyCalculator(10));
      frequencyCalculators.put(RobotSide.RIGHT, new FrequencyCalculator(10));
   }

   public boolean start(String bindAddress)
   {
      if (running)
      {
         throw new RuntimeException(getClass().getName() + " is already running");
      }

      this.bindAddress = new InetSocketAddress(bindAddress, StereoVisionUDPPacketUtil.UDP_PORT);

      try
      {
         socket = new DatagramSocket(this.bindAddress);
         socket.setSoTimeout(1000);
      }
      catch (SocketException e)
      {
         LogTools.error(e);
         return false;
      }

      running = true;

      receiveThread = new Thread(() ->
      {
         while (running && !socket.isClosed())
            StereoVisionUDPPacketUtil.socketReceive(socket, this);
      }, getClass().getSimpleName() + "UDPReceiveThread");
      receiveThread.start();

      return running;
   }

   public void stop()
   {
      running = false;

      if (socket != null)
      {
         socket.close();
      }

      if (receiveThread != null)
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

      receiveThread = null;
      socket = null;
   }

   @Override
   public void onImageFragmentPacket(StereoVisionImageFragmentPacket imageFragmentPacket)
   {
      RobotSide side = imageFragmentPacket.getRobotSide();

      imageDimensions.get(side).setImageWidth(imageFragmentPacket.getImageWidth());
      imageDimensions.get(side).setImageHeight(imageFragmentPacket.getImageHeight());

      frameNumbers.get(side).set(imageFragmentPacket.getFrameNumber());

      byte[] imageBuffer = imageBuffers.get(side);

      int imageBytes = imageFragmentPacket.getImageDataLength();
      if (imageBuffer == null || imageBuffer.length != imageBytes)
      {
         imageBuffer = new byte[imageBytes];
         imageBuffers.put(side, imageBuffer);
      }

      int fragment = imageFragmentPacket.getFragmentNumber();
      int fragmentDataLength = imageFragmentPacket.getFragmentDataLength();
      int fragmentDataOffset = fragment * fragmentDataLength;
      for (int i = 0; i < fragmentDataLength; i++)
         imageBuffer[fragmentDataOffset + i] = imageFragmentPacket.getData()[i];

      frequencyCalculators.get(side).ping();
   }

   public boolean isConnected()
   {
      return socket != null && socket.isConnected();
   }

   @Nullable
   public InetSocketAddress getBindAddress()
   {
      return bindAddress;
   }

   public SideDependentList<byte[]> getImageBuffers()
   {
      return imageBuffers;
   }

   public SideDependentList<ImageDimensions> getImageDimensions()
   {
      return imageDimensions;
   }

   public SideDependentList<AtomicInteger> getFrameNumbers()
   {
      return frameNumbers;
   }

   public SideDependentList<FrequencyCalculator> getFrequencyCalculators()
   {
      return frequencyCalculators;
   }
}
