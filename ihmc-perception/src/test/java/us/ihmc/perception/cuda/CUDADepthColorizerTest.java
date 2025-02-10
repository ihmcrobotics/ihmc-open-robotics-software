package us.ihmc.perception.cuda;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class CUDADepthColorizerTest
{
   @Test
   public void testColorization() throws Exception
   {
      // Initialize colorizer
      CUDADepthColorizer colorizer = new CUDADepthColorizer();

      // Get a depth image for testing
      Mat cpuDepthImage = readDepthImage();
      GpuMat depthImage = new GpuMat();
      depthImage.upload(cpuDepthImage);

      // Colorize the image
      GpuMat colorizedDepth = colorizer.colorizeDepth(depthImage);

      // De-colorize the image
      GpuMat deColorizedDepth = colorizer.deColorizeDepth(colorizedDepth);
      Mat cpuDeColorizedDepth = new Mat();
      deColorizedDepth.download(cpuDeColorizedDepth);

      // Find difference between de-colorized and original depth image
      double difference = OpenCVTools.averagePixelDifference(cpuDepthImage, cpuDeColorizedDepth);
      assertEquals(0.0, difference, "Average Difference = " + difference);

      cpuDepthImage.close();
      depthImage.close();
      colorizedDepth.close();
      deColorizedDepth.close();
      cpuDeColorizedDepth.close();
      colorizer.destroy();
   }

   private Mat readDepthImage()
   {
      try
      {
         Path path = Paths.get(Objects.requireNonNull(RawImageTest.class.getResource("zedDepth16U.raw")).toURI());
         byte[] imageBytes = Files.readAllBytes(path);
         return new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(imageBytes));
      }
      catch (IOException | URISyntaxException e)
      {
         throw new RuntimeException(e);
      }
   }
}
