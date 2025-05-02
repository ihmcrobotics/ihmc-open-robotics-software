package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.FileStorage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class PerceptionLoggingTools
{
   public static void logHeightMap(PerceptionDataLogger perceptionDataLogger, Mat heightMapImage, String namespace)
   {
      BytePointer compressedDepthPointer = new BytePointer();
      OpenCVTools.compressImagePNG(heightMapImage, compressedDepthPointer);
      perceptionDataLogger.storeBytesFromPointer(namespace, compressedDepthPointer);
      compressedDepthPointer.deallocate();
   }

   public static void saveDepthImageToFile(Mat depthImage)
   {
      byte[] buffer = new byte[(int) (depthImage.total() * depthImage.elemSize())];
      depthImage.data().get(buffer);
      try
      {
         Path outputPath = Paths.get("depthImageAsBytes.raw");
         Files.write(outputPath, buffer);
         System.out.println("Saved to: " + outputPath.toAbsolutePath());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
