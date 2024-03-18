package us.ihmc.avatar.stereoVision.net.udp;

import us.ihmc.avatar.stereoVision.net.packet.StereoVisionImageFragmentPacket;
import us.ihmc.avatar.stereoVision.net.packet.StereoVisionPacket;
import us.ihmc.avatar.stereoVision.sensor.StereoVisionSensorReaderListener;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

// On-robot process
public class StereoVisionUDPClient implements StereoVisionSensorReaderListener
{
   @Nullable
   private DatagramSocket socket;
   @Nullable
   private InetAddress sendAddress;
   private volatile boolean running;

   public boolean start(String sendAddress)
   {
      if (running)
      {
         throw new RuntimeException(getClass().getName() + " is already running");
      }

      try
      {
         socket = new DatagramSocket();
      }
      catch (SocketException e)
      {
         LogTools.error(e);
         return false;
      }

      try
      {
         this.sendAddress = InetAddress.getByName(sendAddress);
      }
      catch (UnknownHostException e)
      {
         LogTools.error(e);
         return false;
      }

      return running = true;
   }

   public void stop()
   {
      running = false;
      sendAddress = null;
      if (socket != null)
         socket.close();
      socket = null;
   }

   public void sendPacket(StereoVisionPacket packet)
   {
      if (socket != null && sendAddress != null && running)
         StereoVisionUDPPacketUtil.send(packet, socket, sendAddress);
   }

   @Override
   public void onNewImage(RobotSide side, int frameNumber, int width, int height, byte[] imageData)
   {
      int imageDataLength = imageData.length;
      int numberOfDatagramFragments = (int) Math.ceil((double) imageDataLength / StereoVisionImageFragmentPacket.MAX_FRAGMENT_DATA_LENGTH);

      for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
      {
         int fragmentDataOffset = fragment * StereoVisionImageFragmentPacket.MAX_FRAGMENT_DATA_LENGTH;
         int fragmentDataLength = Math.min(StereoVisionImageFragmentPacket.MAX_FRAGMENT_DATA_LENGTH, imageDataLength - fragmentDataOffset);

         StereoVisionImageFragmentPacket imageFragmentPacket = new StereoVisionImageFragmentPacket();
         imageFragmentPacket.setRobotSide(side);
         imageFragmentPacket.setImageWidth(width);
         imageFragmentPacket.setImageHeight(height);
         imageFragmentPacket.setImageDataLength(imageDataLength);
         imageFragmentPacket.setFrameNumber(frameNumber);
         imageFragmentPacket.setFragmentDataLength(fragmentDataLength);
         byte[] imageFragmentData = new byte[fragmentDataLength];
         for (int i = 0; i < fragmentDataLength; i++)
            imageFragmentData[i] = imageData[fragmentDataOffset + i];
         imageFragmentPacket.setData(imageFragmentData);

         sendPacket(imageFragmentPacket);
      }
   }
}
