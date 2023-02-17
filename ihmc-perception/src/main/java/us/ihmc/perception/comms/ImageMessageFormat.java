package us.ihmc.perception.comms;

import perception_msgs.msg.dds.ImageMessage;

public enum ImageMessageFormat
{
   COLOR_JPEG_RGB8(3),
   COLOR_PNG_RGB8(3), // TODO: Implement receiver and visualizer
   DEPTH_PNG_16UC1(2),
   ;

   public static final ImageMessageFormat[] values = values();

   private final int bytesPerPixel;

   ImageMessageFormat(int bytesPerPixel)
   {
      this.bytesPerPixel = bytesPerPixel;
   }

   public static ImageMessageFormat getFormat(ImageMessage imageMessage)
   {
      for (ImageMessageFormat imageMessageFormat : values)
      {
         if (imageMessage.getFormat() == imageMessageFormat.ordinal())
         {
            return imageMessageFormat;
         }
      }

      throw new RuntimeException("Missing format " + imageMessage.getFormat());
   }

   public int getBytesPerPixel()
   {
      return bytesPerPixel;
   }
}
