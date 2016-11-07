package us.ihmc.avatar.ros.messages;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.nio.ByteBuffer;

import us.ihmc.utilities.ros.RosTools;

public class ImageMessage
{
   int height;
   int width;
   int step;
   private byte[] imageData;

   public void setFromBuffer(ByteBuffer buffer, int size)
   {
      height = buffer.getInt();
      width = buffer.getInt();
      step = buffer.getInt();
      buffer.get(this.imageData, 0, size);
   }

   public void packBufferedImage(BufferedImage image, ColorModel colorModel)
   {
      image = RosTools.bufferedImageFromByteArrayJpeg(colorModel, this.imageData, width, height);
   }
}
