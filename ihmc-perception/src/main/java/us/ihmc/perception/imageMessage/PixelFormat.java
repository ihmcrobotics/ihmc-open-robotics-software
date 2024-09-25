package us.ihmc.perception.imageMessage;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.opencv.global.opencv_core;
import perception_msgs.msg.dds.ImageMessage;

public enum PixelFormat
{
   YUV_I420(1, 1), // YUV420 format
   BGR8(1, 3),    // 24 bits per pixel, in BGR order
   BGRA8(1, 4),   // 32 bits per pixel, in BGRA order
   RGB8(1, 3),    // 24 bits per pixel, in RGB order
   RGBA8(1, 4),   // 32 bits per pixel, in RGBA order
   GRAY8(1, 1),   // monochrome
   GRAY16(2, 1);  // aka depth

   /** Equivalent to {@code elemsize} of the corresponding OpenCV type */
   public final long bytesPerElement;
   /** How many elements represent a pixel. E.g. GRAY has 1 element per pixel, BGR has 3, and BGRA has 4. */
   public final int elementsPerPixel;

   public final long bytesPerPixel;

   PixelFormat(int bytesPerElement, int elementsPerPixel)
   {
      this.bytesPerElement = bytesPerElement;
      this.elementsPerPixel = elementsPerPixel;
      bytesPerPixel = bytesPerElement * (long) elementsPerPixel;
   }

   public void packImageMessage(ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setPixelFormat((byte) ordinal());
   }

   public static PixelFormat fromImageMessage(ImageMessage imageMessage)
   {
      return values()[imageMessage.getPixelFormat()];
   }

   public int toOpenCVType()
   {
      return switch ((int) bytesPerElement)
      {  // TODO: What about floating point types?
         case 1 -> opencv_core.CV_8UC(elementsPerPixel);
         case 2 -> opencv_core.CV_16UC(elementsPerPixel);
         default -> throw new NotImplementedException("Tomasz has been too busy (or lazy) to implement this. Feel free to do it yourself!");
      };
   }
}
