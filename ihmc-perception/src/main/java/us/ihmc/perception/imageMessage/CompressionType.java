package us.ihmc.perception.imageMessage;

import perception_msgs.msg.dds.ImageMessage;

public enum CompressionType
{
   JPEG,
   PNG,
   ZSTD_JPEG_HYBRID,
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
