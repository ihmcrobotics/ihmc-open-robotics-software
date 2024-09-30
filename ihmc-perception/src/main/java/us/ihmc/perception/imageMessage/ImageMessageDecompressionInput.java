package us.ihmc.perception.imageMessage;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;

import java.nio.ByteBuffer;

/**
 * This class is used to streamline feeding image data from ROS 2
 * into OpenCV. Since it's got some state to prevent allocations,
 * we use a class instead of a static tool method.
 */
public class ImageMessageDecompressionInput
{
   private BytePointer decompressionInputBytePointer = new BytePointer();
   private final Mat decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private boolean matIsUpToDate = false;

   public void extract(ImageMessage imageMessage)
   {
      // Get the buffer
      ByteBuffer dataBuffer = imageMessage.getData().getBuffer();

      if (dataBuffer.isDirect())
      {  // Fastest way to copy a direct ByteBuffer is to construct a new BytePointer using it
         // This will simply copy the address, position, limit, and capacity of the direct buffer with no reallocation.
         decompressionInputBytePointer.close();
         decompressionInputBytePointer = new BytePointer(dataBuffer);
      }
      else
      {  // For non-direct buffers, reallocate only if more space is needed and copy the data
         long messageDataSize = imageMessage.getData().size();

         // Reallocate buffer if it doesn't have enough space
         if (decompressionInputBytePointer.isNull() || decompressionInputBytePointer.capacity() < messageDataSize)
         {  // give 5% more space to avoid small reallocation in the future
            decompressionInputBytePointer.close();
            decompressionInputBytePointer = new BytePointer(messageDataSize + (messageDataSize / 20));
         }

         decompressionInputBytePointer.position(0);
         if (dataBuffer.hasArray())
         {  // If the buffer has a backing array, copy data from that
            decompressionInputBytePointer.put(imageMessage.getData().getBuffer().array(), 0, (int) messageDataSize);
         }
         else
         {  // Otherwise loop over the buffer and copy into the pointer
            for (int i = 0; i < messageDataSize; ++i)
               decompressionInputBytePointer.put(i, dataBuffer.get(i));
         }
      }

      decompressionInputBytePointer.limit(imageMessage.getData().size());
      matIsUpToDate = false;
   }

   public BytePointer getInputPointer()
   {
      return decompressionInputBytePointer;
   }

   public Mat getInputMat()
   {
      if (!matIsUpToDate)
      {
         decompressionInputMat.cols((int) decompressionInputBytePointer.limit());
         decompressionInputMat.data(decompressionInputBytePointer);
         matIsUpToDate = true;
      }

      return decompressionInputMat;
   }

   public void destroy()
   {
      decompressionInputBytePointer.close();
      decompressionInputMat.close();
   }
}
