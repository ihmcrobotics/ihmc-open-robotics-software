package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;

import java.nio.ByteBuffer;

public class ImageMessageDecompressionInput
{
   private ByteBuffer decompressionInputBuffer;
   private BytePointer decompressionInputBytePointer;
   private Mat decompressionInputMat;

   public void setup(int uncompressedImageSizeBytes)
   {
      decompressionInputBuffer = NativeMemoryTools.allocate(uncompressedImageSizeBytes);
      decompressionInputBytePointer = new BytePointer(decompressionInputBuffer);
      decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
   }

   public void extract(ImageMessage imageMessage)
   {
      int numberOfBytes = imageMessage.getData().size();
      PerceptionMessageTools.extractImageMessageData(imageMessage, decompressionInputBuffer);

      decompressionInputBytePointer.position(0);
      decompressionInputBytePointer.limit(numberOfBytes);

      decompressionInputMat.cols(numberOfBytes);
      decompressionInputMat.data(decompressionInputBytePointer);
   }

   public Mat getInputMat()
   {
      return decompressionInputMat;
   }
}
