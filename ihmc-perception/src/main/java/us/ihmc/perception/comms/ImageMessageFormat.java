package us.ihmc.perception.comms;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair.ImageType;

public enum ImageMessageFormat
{
   COLOR_JPEG_YUVI420(3), // We usually compress/decompress this to/from RGB8
   COLOR_JPEG_BGR8(3),
   COLOR_PNG_RGB8(3), // TODO: Implement receiver and visualizer
   COLOR_UNCOMPRESSED_BGR8(3),
   DEPTH_PNG_16UC1(2),
   DEPTH_HYBRID_ZSTD_JPEG_16UC1(2),
   DEPTH_UNCOMPRESSED_16UC1(2),
   GRAY_PNG_8UC1(1)
   ;

   public static final ImageMessageFormat[] values = values();

   private final int bytesPerPixel;

   ImageMessageFormat(int bytesPerPixel)
   {
      this.bytesPerPixel = bytesPerPixel;
   }

   public void packMessageFormat(ImageMessage imageMessage)
   {
      imageMessage.setFormat((byte) ordinal());
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

   public static ImageMessageFormat fromImageType(ImageType imageType)
   {
      return switch (imageType)
      {
         case COLOR -> COLOR_UNCOMPRESSED_BGR8;
         case DEPTH -> DEPTH_UNCOMPRESSED_16UC1;
      };
   }
}
