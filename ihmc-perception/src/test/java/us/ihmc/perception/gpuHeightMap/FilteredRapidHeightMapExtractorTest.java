package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.bytedeco.cuda.global.cudart.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Test that we can compute the average and keep a history.
 * Also ensures that things are printed correctly.
 */
public class FilteredRapidHeightMapExtractorTest
{
   @Test
   public void testGettingAverage()
   {
      int rows = 10;
      int cols = 10;
      int layers = 2;

      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(null, rows, cols, layers);

      for (int i = 0; i < layers; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(i * 2 + 2));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
         Mat temp = new Mat();
         latestDepthMat.download(temp);
         PerceptionDebugTools.printMat("Current", temp, 1);
      }

      filteredRapidHeightMapExtractor.destroy();
   }

   @Test
   public void testChangeAfterSteadyState()
   {
      int rows = 2;
      int cols = 2;
      int layers = 2;

      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(null, rows, cols, layers);

      // This example fills the history with the same value, all 8's
      // Lets see how this behaves when we later than introduce a change (noise)
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(800));
      for (int i = 0; i < layers; i++)
      {
         cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(800));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      }

      Mat cpuDataAdjusted = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(1000));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuDataAdjusted);

      filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      Mat temp = new Mat();
      latestDepthMat.download(temp);
      PerceptionDebugTools.printMat("Current", temp, 1);

      for (int i = 0; i < temp.rows(); i++)
      {
         for (int j = 0; j < temp.cols(); j++)
         {
            // This data should be more than the latest data because it's weighted towards the previous average
            assertTrue(temp.col(i).row(j).data().getShort() > cpuData.col(j).data().getShort());
         }
      }

      filteredRapidHeightMapExtractor.destroy();
   }

   @Test
   public void testChangedAfterSteadyStateRealDepthValuesUnsignedShort()
   {
      int rows = 2;
      int cols = 2;
      int layers = 2;

      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(null, rows, cols, layers);

      // This example fills the history with the same value, all 8's
      // Lets see how this behaves when we later than introduce a change (noise)
      for (int i = 0; i < layers; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(32768));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      }

      // 400 is about 20 centimeters? Ish depending on the parameters, this is hard coded could change it to be based on the parameters
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      Mat temp = new Mat();
      latestDepthMat.download(temp);
      PerceptionDebugTools.printMat("Current", temp, 1);
      PerceptionDebugTools.printMat("Original", cpuData, 1);

      for (int i = 0; i < temp.rows(); i++)
      {
         for (int j = 0; j < temp.cols(); j++)
         {
            // This data should be less than the latest data because it's weighted towards the previous average
            assertTrue(((temp.row(i).col(j).data().getShort() & 0xFFFF) < (cpuData.row(i).col(j).data().getShort() & 0xFFFF)));
         }
      }

      filteredRapidHeightMapExtractor.destroy();
   }

   /**
    * This test is meant to run locally to see if there is a GPU memory leak.
    * We want to run the update method of the {@link FilteredRapidHeightMapExtractor} to see if the GPU memory usage is increasing.
    * This makes have problems when another process is running and allocating memory on the GPU.
    * To ensure correct performance, don't run anything else on the GPU
    */
   @Test
   @Disabled
   public void testGPUMemoryUsage()
   {
      // Set a decent size for the rows and cols to make it easier to see a memory leak
      int rows = 1000;
      int cols = 1000;
      int layers = 5;

      FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(null, rows, cols, layers);

      // The value here doesn't matter just needs to fill it with something in order to run the update method.
      for (int i = 0; i < layers; i++)
      {
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(32768));
         GpuMat latestDepthMat = new GpuMat();
         latestDepthMat.upload(cpuData);

         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);
      }

      // Our data to pass into the update call over and over again.
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      // Run this over and over to see if there is a memory leak
      for (int i = 0; i < 100000; i++)
      {
         filteredRapidHeightMapExtractor.update(latestDepthMat, 0);

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
   }
}
