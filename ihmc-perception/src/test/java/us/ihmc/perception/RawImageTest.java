package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;

import java.time.Instant;

import static org.junit.jupiter.api.Assertions.*;

public class RawImageTest
{
   private static final int WIDTH = 128;
   private static final int HEIGHT = 128;
   private static final int THREAD_COUNT = 16;

   @Test
   public void testCPUtoGPUUpload()
   {
      try (Mat cpuMat = new Mat(WIDTH, HEIGHT, opencv_core.CV_8UC3, new Scalar(0.0, 255.0, 0.0, 0.0)))
      {
         RawImage cpuImage = new RawImage(0L,
                                          Instant.now(),
                                          WIDTH,
                                          HEIGHT,
                                          0.0f,
                                          cpuMat,
                                          null,
                                          opencv_core.CV_8UC3,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          new FramePoint3D(),
                                          new FrameQuaternion());

         GpuMat[] gpuMats = new GpuMat[THREAD_COUNT];
         for (int i = 0; i < THREAD_COUNT; ++i)
         {
            final int threadIndex = i;
            ThreadTools.startAThread(() -> gpuMats[threadIndex] = cpuImage.getGpuImageMat(), "TestThread" + i);
         }

         // bad way to ensure all threads finish
         ThreadTools.sleep(100);

         for (int i = 0; i < THREAD_COUNT; ++i)
         {
            assertFalse(gpuMats[i].isNull());
            assertFalse(gpuMats[i].empty());
         }

         cpuImage.release();
      }
   }

   @Test
   public void testGPUtoCPUDownload()
   {
      try (Mat cpuMat = new Mat(WIDTH, HEIGHT, opencv_core.CV_8UC3, new Scalar(0.0, 255.0, 0.0, 0.0));
           GpuMat gpuMat = new GpuMat())
      {
         gpuMat.upload(cpuMat);
         RawImage gpuImage = new RawImage(0L,
                                          Instant.now(),
                                          WIDTH,
                                          HEIGHT,
                                          0.0f,
                                          null,
                                          gpuMat,
                                          opencv_core.CV_8UC3,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          new FramePoint3D(),
                                          new FrameQuaternion());

         Mat[] cpuMats = new Mat[THREAD_COUNT];
         for (int i = 0; i < THREAD_COUNT; ++i)
         {
            final int threadIndex = i;
            ThreadTools.startAThread(() -> cpuMats[threadIndex] = gpuImage.getCpuImageMat(), "TestThread" + i);
         }

         // bad way to ensure all threads finish
         ThreadTools.sleep(100);

         for (int i = 0; i < THREAD_COUNT; ++i)
         {
            assertFalse(cpuMats[i].isNull());
            assertFalse(cpuMats[i].empty());
         }

         gpuImage.release();
      }
   }
}
