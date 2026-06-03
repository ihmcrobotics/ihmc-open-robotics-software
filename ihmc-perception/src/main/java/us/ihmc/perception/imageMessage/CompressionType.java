package us.ihmc.perception.imageMessage;

import perception_msgs.ImageMessage;

public enum CompressionType
{
   JPEG,
   PNG,
   NVJPEG, // NVJPEG compression and OpenCV JPEG compression are not compatible. WHY NVIDIA, WHY?????
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
