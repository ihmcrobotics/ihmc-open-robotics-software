package us.ihmc.darpaRoboticsChallenge.ros.messages;

import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosTools;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.nio.ByteBuffer;

public class CompressedImageMessage
{
   private byte[] imageData;

   public void setFromBuffer(ByteBuffer buffer)
   {
      imageData = buffer.array();
   }

   public void packBufferedImage(BufferedImage image, ColorModel colorModel)
   {
      image = RosTools.bufferedImageFromByteArrayJpeg(colorModel, this.imageData);
   }
}
