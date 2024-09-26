package us.ihmc.perception.imageMessage;

import perception_msgs.msg.dds.ImageMessage;

public enum CompressionType
{
   JPEG,
   NVJPEG, // NVJPEG compression and OpenCV JPEG compression are not compatible. WHY NVIDIA, WHY?????
   PNG,
   NVCOMP,
   ZSTD_NVJPEG_HYBRID,
   UNCOMPRESSED;

   public void packImageMessage(ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setCompressionType((byte) ordinal());
   }

   public static CompressionType fromImageMessage(ImageMessage imageMessage)
   {
      return values()[imageMessage.getCompressionType()];
   }
}
