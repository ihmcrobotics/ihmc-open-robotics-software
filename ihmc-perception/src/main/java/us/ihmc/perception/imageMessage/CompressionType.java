package us.ihmc.perception.imageMessage;

import perception_msgs.ImageMessage;

public enum CompressionType
{
   JPEG,
   PNG,
   NVJPEG, // NVJPEG compression and OpenCV JPEG compression are not compatible. WHY NVIDIA, WHY?????
   UNCOMPRESSED,
   UNKNOWN;

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static CompressionType fromByte(byte compressionTypeAsByte)
   {
      int ordinal = compressionTypeAsByte & 0xFF;
      CompressionType[] values = values();
      if (ordinal >= values.length)
         return UNKNOWN;
      return values[ordinal];
   }

   public static CompressionType fromImageMessage(ImageMessage imageMessage)
   {
      return fromByte(imageMessage.getCompressionType());
   }
}
