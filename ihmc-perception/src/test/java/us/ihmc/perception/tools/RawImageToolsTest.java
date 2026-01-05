package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class RawImageToolsTest
{
   private static final boolean DISPLAY_COMPARISON = false;
   private static final CameraIntrinsics ZED_LEFT_CAMERA_INTRINSICS = new CameraIntrinsics(720, 1280, 531.54, 531.215, 633.1, 360.175);

   private static RawImage cpuBGRImage;
   private static RawImage gpuBGRImage;
   private static RawImage cpuDepthImage;
   private static RawImage gpuDepthImage;

   @BeforeAll
   public static void initializeImages() throws URISyntaxException, IOException
   {  // Initialize images in various pixel formats
      Path zedColorBGRPath = Path.of(Objects.requireNonNull(RawImageTest.class.getResource("zedColorBGR.raw")).toURI());
      byte[] colorBytes = Files.readAllBytes(zedColorBGRPath);

      Mat bgrMat = new Mat(720, 1280, opencv_core.CV_8UC3, new BytePointer(colorBytes));
      cpuBGRImage = RawImage.createWithBGRImage(bgrMat, ZED_LEFT_CAMERA_INTRINSICS, new RigidBodyTransform(), Instant.EPOCH, 0);

      GpuMat gpuBGRMat = new GpuMat();
      gpuBGRMat.upload(bgrMat);
      gpuBGRImage = RawImage.createWithBGRImage(gpuBGRMat, ZED_LEFT_CAMERA_INTRINSICS, new RigidBodyTransform(), Instant.EPOCH, 0);


      Path zedDepth16UPath = Path.of(Objects.requireNonNull(RawImageTest.class.getResource("zedDepth16U.raw")).toURI());
      byte[] depthBytes = Files.readAllBytes(zedDepth16UPath);
      Mat depthMat = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(depthBytes));
      cpuDepthImage = RawImage.createWith16BitDepth(depthMat, ZED_LEFT_CAMERA_INTRINSICS, new RigidBodyTransform(), Instant.EPOCH, 0, 1000.0f);

      GpuMat gpuDepthMat = new GpuMat();
      gpuDepthMat.upload(depthMat);
      gpuDepthImage = RawImage.createWith16BitDepth(gpuDepthMat, ZED_LEFT_CAMERA_INTRINSICS, new RigidBodyTransform(), Instant.EPOCH, 0, 1000.0f);
   }

   @AfterAll
   public static void releaseImages()
   {
      cpuBGRImage.release();
      gpuBGRImage.release();
      cpuDepthImage.release();
      gpuDepthImage.release();
   }

   @Test
   public void test()
   {
      try
      {
         Loader.load(opencv_cudawarping.class);
      }
      catch (Throwable e)
      {
         System.out.println(e.getMessage());
         fail();
      }
   }

   @Test
   public void testResize()
   {
      testResize(cpuBGRImage, 640, 360, 2.0);
      testResize(cpuBGRImage, 1000, 500, 1.5);
      testResize(cpuBGRImage, 2000, 1000, 1.0);
      testResize(cpuBGRImage, 2560, 1440, 1.0);

      testResize(gpuBGRImage, 640, 360, 2.0);
      testResize(gpuBGRImage, 1000, 500, 1.5);
      testResize(gpuBGRImage, 2000, 1000, 1.0);
      testResize(gpuBGRImage, 2560, 1440, 1.0);

      testResize(cpuDepthImage, 640, 360, 25.0);
      testResize(cpuDepthImage, 1000, 500, 15.0);
      testResize(cpuDepthImage, 2000, 1000, 1E-6);
      testResize(cpuDepthImage, 2560, 1440, 1E-6);

      testResize(gpuDepthImage, 640, 360, 25.0);
      testResize(gpuDepthImage, 1000, 500, 15.0);
      testResize(gpuDepthImage, 2000, 1000, 1E-6);
      testResize(gpuDepthImage, 2560, 1440, 1E-6);

      // Test for no resize
      assertEquals(1, cpuBGRImage.getReferenceCount());

      RawImage sameSizeBGR = RawImageTools.resize(cpuBGRImage, cpuBGRImage.getWidth(), cpuBGRImage.getHeight());

      assertEquals(cpuBGRImage, sameSizeBGR);
      assertEquals(2, sameSizeBGR.getReferenceCount());

      sameSizeBGR.release();

      assertEquals(1, cpuBGRImage.getReferenceCount());
   }

   private void testResize(RawImage inputImage, int newWidth, int newHeight, double acceptableDifference)
   {
      int inputReferencesBefore = inputImage.getReferenceCount();
      boolean hasCpuMatBefore = inputImage.hasCpuImage();
      boolean hasGpuMatBefore = inputImage.hasGpuImage();

      // Resize the image
      RawImage resizedImage = RawImageTools.resize(inputImage, newWidth, newHeight);

      // Display the comparison of input vs resized image
      displayComparison(inputImage, resizedImage);

      // Assert several things
      assertEquals(newWidth, resizedImage.getWidth());
      assertEquals(newHeight, resizedImage.getHeight());
      assertEquals(newWidth, resizedImage.getCpuImageMat().cols());
      assertEquals(newHeight, resizedImage.getCpuImageMat().rows());
      assertEquals(1, resizedImage.getReferenceCount());
      assertNotEquals(inputImage, resizedImage);

      assertEquals(inputReferencesBefore, inputImage.getReferenceCount());
      assertEquals(hasCpuMatBefore, inputImage.hasCpuImage());
      assertEquals(hasGpuMatBefore, inputImage.hasGpuImage());

      // Resize the image back to its original size to compare pixels (should be about the same)
      RawImage originalSizeImage = RawImageTools.resize(resizedImage, inputImage.getWidth(), inputImage.getHeight());
      double difference = OpenCVTools.averagePixelDifference(inputImage.getCpuImageMat(), originalSizeImage.getCpuImageMat());
      LogTools.info("Average Pixel Difference: {}", difference);
      assertTrue(difference < acceptableDifference);

      resizedImage.release();
      originalSizeImage.release();
   }

   @Test
   public void testScale()
   {
      testScale(cpuBGRImage, 0.5, 0.5, 2.0);
      testScale(cpuBGRImage, 0.9, 0.8, 1.0);
      testScale(cpuBGRImage, 1.5, 1.3, 1.0);
      testScale(cpuBGRImage, 2.0, 2.0, 1.0);

      testScale(gpuBGRImage, 0.5, 0.5, 2.0);
      testScale(gpuBGRImage, 0.9, 0.8, 1.0 );
      testScale(gpuBGRImage, 1.5, 1.3, 1.0);
      testScale(gpuBGRImage, 2.0, 2.0, 1.0);

      testScale(cpuDepthImage, 0.5, 0.5, 25.0);
      testScale(cpuDepthImage, 0.9, 0.8, 15.0);
      testScale(cpuDepthImage, 1.5, 1.3, 1E-6);
      testScale(cpuDepthImage, 2.0, 2.0, 1E-6);

      testScale(gpuDepthImage, 0.5, 0.5, 25.0);
      testScale(gpuDepthImage, 0.9, 0.8, 15.0);
      testScale(gpuDepthImage, 1.5, 1.3, 1E-6);
      testScale(gpuDepthImage, 2.0, 2.0, 1E-6);

      // Test for no resize
      assertEquals(1, cpuBGRImage.getReferenceCount());

      RawImage sameSizeBGR = RawImageTools.scale(cpuBGRImage, 1.0, 1.0);

      assertEquals(cpuBGRImage, sameSizeBGR);
      assertEquals(2, sameSizeBGR.getReferenceCount());

      sameSizeBGR.release();

      assertEquals(1, cpuBGRImage.getReferenceCount());
   }

   private void testScale(RawImage inputImage, double scaleX, double scaleY, double acceptableDifference)
   {
      int inputReferencesBefore = inputImage.getReferenceCount();
      boolean hasCpuMatBefore = inputImage.hasCpuImage();
      boolean hasGpuMatBefore = inputImage.hasGpuImage();

      // Scale the image
      RawImage scaledImage = RawImageTools.scale(inputImage, scaleX, scaleY);

      // Display the comparison of input vs scaled image
      displayComparison(inputImage, scaledImage);

      // Assert several things
      int expectedWidth = (int) Math.round(scaleX * inputImage.getWidth());
      int expectedHeight = (int) Math.round(scaleY * inputImage.getHeight());

      assertEquals(expectedWidth, scaledImage.getWidth());
      assertEquals(expectedHeight, scaledImage.getHeight());
      assertEquals(expectedWidth, scaledImage.getCpuImageMat().cols());
      assertEquals(expectedHeight, scaledImage.getCpuImageMat().rows());
      assertEquals(1, scaledImage.getReferenceCount());
      assertNotEquals(inputImage, scaledImage);

      assertEquals(inputReferencesBefore, inputImage.getReferenceCount());
      assertEquals(hasCpuMatBefore, inputImage.hasCpuImage());
      assertEquals(hasGpuMatBefore, inputImage.hasGpuImage());

      // Resize the image back to its original size to compare pixels (should be about the same)
      RawImage originalSizeImage = RawImageTools.resize(scaledImage, inputImage.getWidth(), inputImage.getHeight());
      double difference = OpenCVTools.averagePixelDifference(inputImage.getCpuImageMat(), originalSizeImage.getCpuImageMat());
      LogTools.info("Average Pixel Difference: {}", difference);
      assertTrue(difference < acceptableDifference);

      scaledImage.release();
      originalSizeImage.release();
   }

   private void displayComparison(RawImage imageA, RawImage imageB)
   {
      if (DISPLAY_COMPARISON)
      {
         Mat matA = imageA.getCpuImageMat();
         Mat matB = imageB.getCpuImageMat();

         int largerWidth = Math.max(imageA.getWidth(), imageB.getWidth());
         int largerHeight = Math.max(imageA.getHeight(), imageB.getHeight());

         try (Mat paddedA = new Mat();
              Mat paddedB = new Mat();
              Scalar zero = new Scalar(0))
         {
            int verticalBorderSizeA = (largerWidth - matA.cols()) / 2;
            int horizontalBorderSizeA = (largerHeight - matA.rows()) / 2;
            opencv_core.copyMakeBorder(matA,
                                       paddedA,
                                       horizontalBorderSizeA,
                                       horizontalBorderSizeA,
                                       verticalBorderSizeA,
                                       verticalBorderSizeA,
                                       opencv_core.BORDER_CONSTANT,
                                       zero);

            int verticalBorderSizeB = (largerWidth - matB.cols()) / 2;
            int horizontalBorderSizeB = (largerHeight - matB.rows()) / 2;
            opencv_core.copyMakeBorder(matB,
                                       paddedB,
                                       horizontalBorderSizeB,
                                       horizontalBorderSizeB,
                                       verticalBorderSizeB,
                                       verticalBorderSizeB,
                                       opencv_core.BORDER_CONSTANT,
                                       zero);

            MatVector images = new MatVector(paddedA, paddedB);
            PerceptionDebugTools.display("Comparison", images, 3000);
         }
      }
   }
}
