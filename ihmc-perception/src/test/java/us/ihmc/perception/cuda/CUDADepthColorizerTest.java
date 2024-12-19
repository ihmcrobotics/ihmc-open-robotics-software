package us.ihmc.perception.cuda;

import org.apache.commons.io.IOUtils;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.IOException;
import java.net.URL;

import static org.junit.jupiter.api.Assertions.*;

public class CUDADepthColorizerTest
{
   @Test
   public void testColorization()
   {
      CUDADepthColorizer colorizer = new CUDADepthColorizer();
      Mat cpuDepthImage = readDepthImage();
      GpuMat depthImage = new GpuMat();
      depthImage.upload(cpuDepthImage);

      GpuMat colorizedDepth = colorizer.colorizeDepth(depthImage);
      Mat cpuColorizedDepth = new Mat();
      colorizedDepth.download(cpuColorizedDepth);

      GpuMat deColorizedDepth = colorizer.deColorizeDepth(colorizedDepth);
      Mat cpuDeColorizedDepth = new Mat();
      deColorizedDepth.download(cpuDeColorizedDepth);

      double difference = OpenCVTools.averagePixelDifference(cpuDepthImage, cpuDeColorizedDepth);
      assertTrue(difference < 1.0, "Average Difference = " + difference);

      depthImage.close();
      colorizer.destroy();
   }

   private Mat readDepthImage()
   {
      try
      {
         URL imageURL = RawImageTest.class.getResource("zedDepth16U.raw");
         byte[] imageBytes = IOUtils.toByteArray(imageURL);
         return new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(imageBytes));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
