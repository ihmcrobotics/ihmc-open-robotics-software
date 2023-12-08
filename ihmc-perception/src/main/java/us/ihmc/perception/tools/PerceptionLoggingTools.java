package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.opencv.OpenCVTools;

public class PerceptionLoggingTools
{
   public static void logHeightMap(PerceptionDataLogger perceptionDataLogger, Mat heightMapImage, String namespace)
   {
      BytePointer compressedDepthPointer = new BytePointer();
      OpenCVTools.compressImagePNG(heightMapImage, compressedDepthPointer);
      perceptionDataLogger.storeBytesFromPointer(namespace, compressedDepthPointer);
      compressedDepthPointer.deallocate();
   }
}
