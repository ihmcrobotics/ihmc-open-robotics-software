package us.ihmc.perception.filters;

import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.bytedeco.cuda.global.cudart.cudaFree;
import static org.bytedeco.cuda.global.cudart.cudaMemGetInfo;
import static org.junit.jupiter.api.Assertions.*;

public class CUDAFlyingPointsFilterTest
{
   // Tests a simple 3x3 matrix where the input contains an outlier.
   // The test validates whether the kernel correctly replaces the outlier with the median of the surrounding values.
   @Test
   public void testSimpleMatrix()
   {
      DepthImageFlyingPointsFilter flyingPointsFilter;
      Mat outputMat = new Mat();
      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();
      flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);

      Mat inputMat = new Mat(3, 3, opencv_core.CV_16UC1);
      inputMat.ptr(0, 0).putShort((short) 10);
      inputMat.ptr(0, 1).putShort((short) 10);
      inputMat.ptr(0, 2).putShort((short) 10);
      inputMat.ptr(1, 0).putShort((short) 10);
      inputMat.ptr(1, 1).putShort((short) 15); // outlier
      inputMat.ptr(1, 2).putShort((short) 10);
      inputMat.ptr(2, 0).putShort((short) 10);
      inputMat.ptr(2, 1).putShort((short) 10);
      inputMat.ptr(2, 2).putShort((short) 10);

      PerceptionDebugTools.printMat("input_matrix", inputMat, 1);

      GpuMat deviceInputMat = new GpuMat(inputMat.size(), inputMat.type());
      deviceInputMat.upload(inputMat);
      GpuMat deviceOutputMat = new GpuMat(deviceInputMat.size(), deviceInputMat.type());

      flyingPointsFilter.applyFilter(deviceInputMat, deviceOutputMat, new CameraIntrinsics());
      deviceInputMat.close();

      PerceptionDebugTools.printMat("output_matrix", outputMat, 1);

      // Validate that all elements in outputMat are 10
      for (int i = 0; i < outputMat.rows(); i++)
      {
         for (int j = 0; j < outputMat.cols(); j++)
         {
            assertEquals(10, outputMat.ptr(i, j).get(), "Element [" + i + "][" + j + "]");
         }
      }

      deviceOutputMat.close();
      outputMat.close();
      flyingPointsFilter.destroy();
   }

   @Test
   @Disabled
   public void testGPUMemoryUsage()
   {
      // Set a decent size for the rows and cols to make it easier to see a memory leak
      int rows = 1000;
      int cols = 1000;
      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();
      DepthImageFlyingPointsFilter flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);

      // Our data to pass into the update call over and over again.
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat deviceInputData = new GpuMat();
      deviceInputData.upload(cpuData);

      GpuMat deviceOutputData = new GpuMat(deviceInputData.size(), deviceInputData.type());

      // Run this over and over to see if there is a memory leak
      for (int i = 0; i < 10000; i++)
      {
         flyingPointsFilter.applyFilter(deviceInputData, deviceOutputData, new CameraIntrinsics());

         SizeTPointer freePointer = new SizeTPointer(1);
         SizeTPointer usedPointer = new SizeTPointer(1);

         cudaMemGetInfo(freePointer, usedPointer);

         // GPU Memory information
         long freeMemory = freePointer.get();
         long totalMemory = usedPointer.get();
         long usedMemory = totalMemory - freeMemory;

         System.out.println("Free Memory:  " + freeMemory);
         System.out.println("Total Memory: " + totalMemory);
         System.out.println("Used memory:  " + usedMemory);

         cudaFree(freePointer);
         cudaFree(usedPointer);
      }

      deviceInputData.close();
      deviceOutputData.close();

      cpuData.close();
      flyingPointsFilter.destroy();
   }
}