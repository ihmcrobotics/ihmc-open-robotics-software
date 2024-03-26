package us.ihmc.avatar.stereoVision.net.udp;

import us.ihmc.avatar.stereoVision.net.packet.StereoVisionImageFragmentPacket;
import us.ihmc.avatar.stereoVision.net.packet.StereoVisionPacketListener;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyCalculator;

// UI process
public class StereoVisionUDPServer implements StereoVisionPacketListener
{
   private final SideDependentList<byte[]> imageBuffers = new SideDependentList<>();
   private final SideDependentList<ImageDimensions> imageDimensions = new SideDependentList<>(ImageDimensions::new);
   private final SideDependentList<FrequencyCalculator> frequencyCalculators;

   public StereoVisionUDPServer()
   {
      frequencyCalculators = new SideDependentList<>();
      frequencyCalculators.put(RobotSide.LEFT, new FrequencyCalculator(10));
      frequencyCalculators.put(RobotSide.RIGHT, new FrequencyCalculator(10));
   }

   public void start()
   {

   }

   public void stop()
   {

   }

   @Override
   public void onImageFragmentPacket(StereoVisionImageFragmentPacket imageFragmentPacket)
   {
      RobotSide side = imageFragmentPacket.getRobotSide();

      imageDimensions.get(side).setImageWidth(imageFragmentPacket.getImageWidth());
      imageDimensions.get(side).setImageHeight(imageFragmentPacket.getImageHeight());

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
}
