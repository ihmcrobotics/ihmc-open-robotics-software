package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.tools.thread.MissingThreadTools;

import java.time.Instant;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class RawImageTest
{
   private final Random random = new Random(0);
   private final Mat mat8UC1 = new Mat(10, 10, opencv_core.CV_8U);
   private final Mat mat8UC3 = new Mat(10, 10, opencv_core.CV_8UC3);
   private final Mat mat16UC1 = new Mat(5, 5, opencv_core.CV_16UC1);
   private final GpuMat gpuMat8UC1 = new GpuMat(10, 10, opencv_core.CV_8U);

   @BeforeEach
   public void initializeMats()
   {
      for (int i = 0; i < mat8UC1.data().limit(); i += 8)
         mat8UC1.data().put(i, (byte)((i * i) % 256));
      gpuMat8UC1.upload(mat8UC1);

      for (int i = 0; i < mat8UC3.data().limit(); i += 8)
         mat8UC3.data().put(i, (byte)((i * i) % 256));

      for (int i = 0; i < mat16UC1.data().limit(); i += 8)
         mat16UC1.data().put(i, (byte)((i * i) % 256));
   }

   @AfterEach
   public void closeMats()
   {
      mat8UC1.close();
      gpuMat8UC1.close();
   }

   @Test
   public void testReferenceCount() throws InterruptedException
   {
      RawImage testImage = createRawImage(mat8UC1);

      int numThreads = 50;
      Thread[] threads = new Thread[numThreads];
      for (int i = 0; i < numThreads; ++i)
      {
         threads[i] = new Thread(() ->
         {
            assertNotNull(testImage.get());
            MissingThreadTools.sleep(random.nextDouble(0.0, 1.0));
            testImage.release();
         });
      }

      for (int i = 0; i < numThreads; ++i)
      {
         threads[i].join();
      }

      testImage.release();
      assertNull(testImage.get());
   }

   @Test
   public void testCpuGpuImageTransfer()
   {
      // CPU to GPU
      RawImage cpuRawImage = createRawImage(mat8UC1);
      GpuMat gpuCopy = cpuRawImage.getGpuImageMat();
      assertNotNull(gpuCopy);
      assertFalse(gpuCopy.isNull());
      assertFalse(gpuCopy.empty());
      assertTrue(dimensionsMatch(cpuRawImage.getCpuImageMat(), cpuRawImage.getGpuImageMat()));
      cpuRawImage.release();

      // GPU to CPU
      RawImage gpuRawImage = createRawImage(gpuMat8UC1);
      Mat cpuCopy = gpuRawImage.getCpuImageMat();
      assertNotNull(cpuCopy);
      assertFalse(cpuCopy.isNull());
      assertFalse(cpuCopy.empty());
      assertTrue(dimensionsMatch(gpuRawImage.getCpuImageMat(), gpuRawImage.getGpuImageMat()));
      gpuRawImage.release();
   }

   @Test
   public void testReplaceImageCPUtoGPU()
   {
      // CPU to GPU change
      assertDoesNotThrow(() ->
      {
         RawImage originalImage = createRawImage(mat8UC1);
         RawImage replacedImage = originalImage.replaceImage(gpuMat8UC1);
         assertNotEquals(originalImage, replacedImage);
         assertTrue(dataEquals(originalImage.getCpuImageMat().data(), replacedImage.getCpuImageMat().data()));
         originalImage.release();
         replacedImage.release();
      });
   }

   @Test
   public void testReplaceImageGPUtoCPU()
   {
      // GPU to CPU change
      assertDoesNotThrow(() ->
      {
         RawImage originalImage = createRawImage(gpuMat8UC1);
         RawImage replacedImage = originalImage.replaceImage(mat8UC1);
         assertNotEquals(originalImage, replacedImage);
         assertTrue(dataEquals(originalImage.getCpuImageMat().data(), replacedImage.getCpuImageMat().data()));
         originalImage.release();
         replacedImage.release();
      });
   }

   @Test
   public void testReplaceImageDifferentType()
   {

      // CPU to CPU, different type
      assertDoesNotThrow(() ->
      {
         RawImage originalImage = createRawImage(mat8UC1);
         RawImage replacedImage = originalImage.replaceImage(mat8UC3);
         assertNotEquals(originalImage, replacedImage);
         assertTrue(dataEquals(originalImage.getCpuImageMat().data(), replacedImage.getCpuImageMat().data()));
         originalImage.release();
         replacedImage.release();
      });
   }

   @Test
   public void testReplaceImageDifferentSize()
   {
      // Different dimensions
      assertThrows(IllegalArgumentException.class, () ->
      {
         RawImage originalImage = createRawImage(mat8UC1);
         RawImage replacedImage = originalImage.replaceImage(mat16UC1);
         originalImage.release();
         replacedImage.release();
      });
   }

   private boolean dataEquals(BytePointer dataA, BytePointer dataB)
   {
      if (dataA.limit() != dataB.limit())
         return false;

      for (long i = 0; i < dataA.limit(); i += 8)
      {
         if (dataA.getShort(i) != dataB.getShort(i))
            return false;
      }

      return true;
   }

   private boolean dimensionsMatch(Mat matA, GpuMat matB)
   {
      return matA.cols() == matB.cols() && matA.rows() == matB.rows() && matA.type() == matB.type();
   }

   private RawImage createRawImage(Mat mat)
   {
      return new RawImage(0,
                          Instant.now(),
                          0.0f,
                          mat,
                          null,
                          10.0f,
                          10.0f,
                          mat.cols() / 2.0f,
                          mat.rows() / 2.0f,
                          new FramePoint3D(ReferenceFrame.getWorldFrame()),
                          new FrameQuaternion(ReferenceFrame.getWorldFrame()));
   }

   private RawImage createRawImage(GpuMat mat)
   {
      return new RawImage(0,
                          Instant.now(),
                          0.0f,
                          null,
                          gpuMat8UC1,
                          10.0f,
                          10.0f,
                          mat.cols() / 2.0f,
                          mat.rows() / 2.0f,
                          new FramePoint3D(ReferenceFrame.getWorldFrame()),
                          new FrameQuaternion(ReferenceFrame.getWorldFrame()));
   }
}
