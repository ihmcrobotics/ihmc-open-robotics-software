package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import static org.bytedeco.cuda.global.cudart.cudaFree;
import static org.bytedeco.cuda.global.cudart.cudaMemGetInfo;

public class FilteredVerticalSurfacesExtractorTest
{

   private CUstream_st steam;

   /**
    * This test is meant to run locally to see if there is a GPU memory leak.
    * We want to run the update method of the {@link FilteredVerticalSurfacesExtractor} to see if the GPU memory usage is increasing.
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

      FilteredVerticalSurfacesExtractor filteredVerticalSurfacesExtractor = new FilteredVerticalSurfacesExtractor(null, rows, cols);

      // Our data to pass into the update call over and over again.
      Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(33100));
      GpuMat latestDepthMat = new GpuMat();
      latestDepthMat.upload(cpuData);

      // Run this over and over to see if there is a memory leak
      for (int i = 0; i < 10000; i++)
      {
         filteredVerticalSurfacesExtractor.update(latestDepthMat);

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
