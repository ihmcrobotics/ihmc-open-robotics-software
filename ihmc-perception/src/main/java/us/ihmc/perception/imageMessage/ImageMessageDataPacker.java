package us.ihmc.perception.imageMessage;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.ImageMessage;
import us.ihmc.perception.tools.PerceptionMessageTools;

import java.nio.ByteBuffer;

/**
 * This class is used to simplify the allocation free packing of our
 * ROS 2 ImageMessage data, which requires copying to JVM heap first.
 *
 * This class is useful when you have a BytePointer to data in direct (native)
 * memory and need to pack that data into an ImageMessage. Since our ROS 2
 * messages use a heap-backed {@link us.ihmc.fastddsjava.cdr.idl.IDLByteSequence}, we
 * preallocate a heap array, copy the native data to it, then copy into the message.
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
      PerceptionMessageTools.packDataArray(imageMessageToPack.getData(), ByteBuffer.wrap(heapByteArrayData, 0, numberOfDataBytes));
   }
}
