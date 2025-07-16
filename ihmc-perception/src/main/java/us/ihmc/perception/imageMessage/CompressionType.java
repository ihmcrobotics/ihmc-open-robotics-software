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

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static CompressionType fromByte(byte compressionTypeAsByte)
   {
      return values()[compressionTypeAsByte];
   }

   public static CompressionType fromImageMessage(ImageMessage imageMessage)
   {
      return fromByte(imageMessage.getCompressionType());
   }
}
