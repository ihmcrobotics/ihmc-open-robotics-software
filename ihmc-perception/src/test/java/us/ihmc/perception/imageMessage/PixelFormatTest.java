package us.ihmc.perception.imageMessage;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class PixelFormatTest
{
   private static Mat bgrImage;
   private static final Map<PixelFormat, Mat> pixelFormatToImageMap = new HashMap<>();
   private static final Map<PixelFormat, GpuMat> pixelFormatToGpuImageMap = new HashMap<>();

   @BeforeAll
   public static void initializeImages() throws URISyntaxException, IOException
   {
      Path zedColorBGRPath = Path.of(RawImageTest.class.getResource("zedColorBGR.raw").toURI());
      byte[] colorBytes = Files.readAllBytes(zedColorBGRPath);
      bgrImage = new Mat(720, 1280, opencv_core.CV_8UC3, new BytePointer(colorBytes));
      pixelFormatToImageMap.put(PixelFormat.BGR8, bgrImage);
      GpuMat gpuBGRImage = new GpuMat();
      gpuBGRImage.upload(bgrImage);
      pixelFormatToGpuImageMap.put(PixelFormat.BGR8, gpuBGRImage);

      Mat bgraImage = new Mat();
      opencv_imgproc.cvtColor(bgrImage, bgraImage, opencv_imgproc.COLOR_BGR2BGRA);
      pixelFormatToImageMap.put(PixelFormat.BGRA8, bgraImage);
      GpuMat gpuBGRAImage = new GpuMat();
      gpuBGRAImage.upload(bgraImage);
      pixelFormatToGpuImageMap.put(PixelFormat.BGRA8, gpuBGRAImage);

      Mat rgbImage = new Mat();
      opencv_imgproc.cvtColor(bgrImage, rgbImage, opencv_imgproc.COLOR_BGR2RGB);
      pixelFormatToImageMap.put(PixelFormat.RGB8, rgbImage);
      GpuMat gpuRGBImage = new GpuMat();
      gpuRGBImage.upload(rgbImage);
      pixelFormatToGpuImageMap.put(PixelFormat.RGB8, gpuRGBImage);

      Mat rgbaImage = new Mat();
      opencv_imgproc.cvtColor(bgrImage, rgbaImage, opencv_imgproc.COLOR_BGR2RGBA);
      pixelFormatToImageMap.put(PixelFormat.RGBA8, rgbaImage);
      GpuMat gpuRGBAImage = new GpuMat();
      gpuRGBAImage.upload(rgbaImage);
      pixelFormatToGpuImageMap.put(PixelFormat.RGBA8, gpuRGBAImage);
   }

   @AfterAll
   public static void destroy()
   {
      for (Mat image : pixelFormatToImageMap.values())
         image.close();

      for (GpuMat gpuImage : pixelFormatToGpuImageMap.values())
         gpuImage.close();
   }

   @AfterEach
   public void printLine()
   {  // Just for pretty log output!
      System.out.println();
   }

   @Test
   public void testRGBAConversions()
   {
      for (PixelFormat pixelFormat : pixelFormatToImageMap.keySet())
      {
         Mat pixelFormatImage = pixelFormatToImageMap.get(pixelFormat);
         testColorConversion(pixelFormat, pixelFormatImage, -1);
      }
   }

   @Test
   public void testAllConversion()
   {
      for (PixelFormat pixelFormatA : pixelFormatToImageMap.keySet())
      {
         Mat sourceImage = pixelFormatToImageMap.get(pixelFormatA);
         GpuMat gpuSourceImage = new GpuMat();
         gpuSourceImage.upload(sourceImage);

         for (PixelFormat pixelFormatB : pixelFormatToImageMap.keySet())
         {
            Mat expectedResult = pixelFormatToImageMap.get(pixelFormatB);
            Mat actualResult = new Mat();
            GpuMat gpuResult = new GpuMat();

            LogTools.info("Testing CPU conversion from {} to {}", pixelFormatA, pixelFormatB);
            assertTrue(pixelFormatA.convertToPixelFormat(sourceImage, actualResult, pixelFormatB));
            double averagePixelDifference = OpenCVTools.averagePixelDifference(expectedResult, actualResult);
            assertEquals(0.0, averagePixelDifference, 1E-6);

            LogTools.info("Testing GPU conversion from {} to {}", pixelFormatA, pixelFormatB);
            assertTrue(pixelFormatA.convertToPixelFormat(gpuSourceImage, gpuResult, pixelFormatB));
            gpuResult.download(actualResult);
            averagePixelDifference = OpenCVTools.averagePixelDifference(expectedResult, actualResult);
            assertEquals(0.0, averagePixelDifference, 1E-6);

            actualResult.close();
            gpuResult.close();
         }

         gpuSourceImage.close();
         System.out.println();
      }
   }

   @Test
   // YUV conversions are not commutative, requiring their own tests
   public void testYUVI420Conversions()
   {
      Mat yuvI420Image = new Mat();
      opencv_imgproc.cvtColor(bgrImage, yuvI420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
      testColorConversion(PixelFormat.YUV_I420, yuvI420Image, opencv_imgproc.COLOR_YUV2RGBA_I420);
      yuvI420Image.close();
   }

   @Test
   public void testGrayConversions()
   {
      Mat grayImage = new Mat();
      opencv_imgproc.cvtColor(bgrImage, grayImage, opencv_imgproc.COLOR_BGR2GRAY);
      testColorConversion(PixelFormat.GRAY8, grayImage, opencv_imgproc.COLOR_GRAY2RGBA);
      grayImage.close();
   }

   private void testColorConversion(PixelFormat pixelFormat, Mat originalImage, int toRGBAConversion)
   {
      GpuMat gpuOriginalImage = new GpuMat();
      gpuOriginalImage.upload(originalImage);

      Mat expectedRGBAResult;
      if (toRGBAConversion < 0)
         expectedRGBAResult = pixelFormatToImageMap.get(PixelFormat.RGBA8).clone();
      else
      {
         expectedRGBAResult = new Mat();
         opencv_imgproc.cvtColor(originalImage, expectedRGBAResult, toRGBAConversion);
      }

      // Test conversion to RGBA
      LogTools.info("Testing CPU Conversion from {} to RGBA", pixelFormat);
      Mat rgbaResultImage = new Mat();
      assertTrue(pixelFormat.convertToRGBA(originalImage, rgbaResultImage));
      double averagePixelDifference = OpenCVTools.averagePixelDifference(expectedRGBAResult, rgbaResultImage);
      assertEquals(0.0, averagePixelDifference, 1E-6);

      LogTools.info("Testing GPU Conversion from {} to RGBA", pixelFormat);
      GpuMat gpuRGBAResultImage = new GpuMat();
      assertTrue(pixelFormat.convertToRGBA(gpuOriginalImage, gpuRGBAResultImage));
      gpuRGBAResultImage.download(rgbaResultImage);
      averagePixelDifference = OpenCVTools.averagePixelDifference(expectedRGBAResult, rgbaResultImage);
      assertEquals(0.0, averagePixelDifference, 1E-6);
      gpuRGBAResultImage.close();
      rgbaResultImage.close();
      expectedRGBAResult.close();

      Mat originalRGBAImage = pixelFormatToImageMap.get(PixelFormat.RGBA8);
      GpuMat gpuOriginalRGBAImage = pixelFormatToGpuImageMap.get(PixelFormat.RGBA8);

      // Test RGBA to YUV
      LogTools.info("Testing CPU Conversion from RGBA to {}", pixelFormat);
      Mat resultImage = new Mat();
      assertTrue(pixelFormat.convertFromRGBA(originalRGBAImage, resultImage));
      averagePixelDifference = OpenCVTools.averagePixelDifference(originalImage, resultImage);
      assertEquals(0.0, averagePixelDifference, 1E-6);

      LogTools.info("Testing GPU Conversion from RGBA to {}", pixelFormat);
      GpuMat gpuResultImage = new GpuMat();
      assertTrue(pixelFormat.convertFromRGBA(gpuOriginalRGBAImage, gpuResultImage));
      gpuResultImage.download(resultImage);
      pixelFormat.convertFromRGBA(originalRGBAImage, resultImage);
      averagePixelDifference = OpenCVTools.averagePixelDifference(originalImage, resultImage);
      assertEquals(0.0, averagePixelDifference, 1E-6);
      resultImage.close();
      gpuResultImage.close();

      gpuOriginalImage.close();
   }
}
