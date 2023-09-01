package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;

/**
 * This class is used to simplify the allocation free packing of our
 * ROS 2 ImageMessage data, which requires copying to JVM heap first.
 *
 * This class is useful when you have a BytePointer to data in direct (native)
 * memory and need to pack that data into an ImageMessage. Since our ROS 2
 * messages use a TByteArrayList which is backed by a heap array, we need
 * to preallocate a heap array, copy the native data to it, then copy
 * it again into the ROS 2 message's internal TByteArrayList (getData).
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

   public void pack(ImageMessage imageMessageToPack, BytePointer imageDataBytePointer)
   {
      int numberOfDataBytes = (int) imageDataBytePointer.limit();
      imageDataBytePointer.get(heapByteArrayData, 0, numberOfDataBytes);
      imageMessageToPack.getData().resetQuick();
      imageMessageToPack.getData().add(heapByteArrayData, 0, numberOfDataBytes);
   }
}
