package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;

/**
 * This class is used to simplify the allocation free packing of our
 * ROS 2 ImageMessage data, which requires copying to JVM heap first.
 */
public class ImageMessageDataPacker
{
   private final byte[] heapByteArrayData;

   public ImageMessageDataPacker(long maximumDataBytes)
   {
      this((int) maximumDataBytes);
   }

   public ImageMessageDataPacker(int maximumDataBytes)
   {
      heapByteArrayData = new byte[maximumDataBytes];
   }

   public void pack(ImageMessage imageMessage, BytePointer imageDataBytePointer)
   {
      int numberOfDataBytes = (int) imageDataBytePointer.limit();
      imageDataBytePointer.get(heapByteArrayData, 0, numberOfDataBytes);
      imageMessage.getData().resetQuick();
      imageMessage.getData().add(heapByteArrayData, 0, numberOfDataBytes);
   }
}
