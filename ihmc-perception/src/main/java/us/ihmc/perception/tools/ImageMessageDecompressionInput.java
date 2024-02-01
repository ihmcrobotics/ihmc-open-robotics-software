package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;

/**
 * This class is used to streamline feeding image data from ROS 2
 * into OpenCV. Since it's got some state to prevent allocations,
 * we use a class instead of a static tool method.
 */
public class ImageMessageDecompressionInput
{
   private BytePointer decompressionInputBytePointer;
   private Mat decompressionInputMat;

   public void setup(int uncompressedImageSizeBytes)
   {
      decompressionInputBytePointer = new BytePointer(uncompressedImageSizeBytes);
      decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
   }

   public void extract(ImageMessage imageMessage)
   {
      int numberOfBytes = imageMessage.getData().size();
      PerceptionMessageTools.copyToBytePointer(imageMessage.getData(), decompressionInputBytePointer);

      decompressionInputMat.cols(numberOfBytes);
      decompressionInputMat.data(decompressionInputBytePointer);
   }

   public Mat getInputMat()
   {
      return decompressionInputMat;
   }
}
