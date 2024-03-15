package us.ihmc.avatar.stereoVision.net.udp;

import us.ihmc.avatar.stereoVision.net.packet.StereoVisionImageFragmentPacket;
import us.ihmc.avatar.stereoVision.sensor.StereoVisionSensorReaderListener;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.avatar.stereoVision.net.udp.StereoVisionUDPProperties.DATAGRAM_MAX_LENGTH;

public class StereoVisionUDPClient implements StereoVisionSensorReaderListener
{
   public void start()
   {

   }

   public void stop()
   {

   }

   public void sendImageFragmentPacket(StereoVisionImageFragmentPacket imageFragmentPacket)
   {

   }

   @Override
   public void onNewImage(RobotSide side, int frameNumber, int width, int height, byte[] imageData)
   {
      int imageDataLength = imageData.length;
      int numberOfDatagramFragments = (int) Math.ceil((double) imageDataLength / DATAGRAM_MAX_LENGTH);

      for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
      {
         StereoVisionImageFragmentPacket imageFragmentPacket = new StereoVisionImageFragmentPacket();

         sendImageFragmentPacket(imageFragmentPacket);
      }
   }
}
