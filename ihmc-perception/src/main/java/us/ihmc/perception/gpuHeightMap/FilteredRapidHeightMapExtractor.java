package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.cudaExtent;
import org.bytedeco.cuda.cudart.cudaPitchedPtr;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.jetbrains.annotations.NotNull;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class FilteredRapidHeightMapExtractor
{
   private final cudaPitchedPtr pointerTo3DArray;
   private int currentIndex;
   int layers = 2;

   private final CUstream_st stream;
   private final int rows;
   private final int cols;
   private final CUDAKernel kernel;
   private final CUDAProgram program;
   private int loopTracker = 0;

   public FilteredRapidHeightMapExtractor(CUstream_st stream, int rows, int cols)
   {
      this.stream = stream;
      this.rows = rows;
      this.cols = cols;

      // Load header and main file
      URL kernelPath = getClass().getResource("FilteredRapidHeightMapExtractor.cu");
      try
      {
         program = new CUDAProgram(kernelPath);
         kernel = program.loadKernel("filterRapidHeightMap");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      pointerTo3DArray = new cudaPitchedPtr();
      cudaExtent extent = make_cudaExtent((long) cols * Short.BYTES, rows, layers);
      int error = cudaMalloc3D(pointerTo3DArray, extent);
      CUDATools.checkCUDAError(error);

      currentIndex = 0;

      for (int i = 0; i < layers; i++)
      {
         // To get the average to be a whole number do (i * 2 + 2)
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(0));

         // Allocate memory for each layer, creating a 3d array
         cudaMemcpy2D(pointerTo3DArray.ptr().position(i * pointerTo3DArray.pitch() * pointerTo3DArray.ysize()),
                      pointerTo3DArray.pitch(),
                      cpuData.data(),
                      cpuData.step(),
                      pointerTo3DArray.xsize(),
                      pointerTo3DArray.ysize(),
                      cudaMemcpyDefault);
      }
   }

   public GpuMat update(GpuMat latestGlobalHeightMap)
   {
      GpuMat heightMapAverage = computerHeightMapHistoryAverage(latestGlobalHeightMap);

      return latestGlobalHeightMap;
   }

   @NotNull
   private GpuMat computerHeightMapHistoryAverage(GpuMat latestGlobalHeightMap)
   {
      // Only want to compute the average if we have the past values to use
      if (loopTracker < layers)
      {
         loopTracker++;

         cudaMemcpy2D(pointerTo3DArray.ptr().position(currentIndex * pointerTo3DArray.pitch() * pointerTo3DArray.ysize()),
                      pointerTo3DArray.pitch(),
                      latestGlobalHeightMap.data(),
                      latestGlobalHeightMap.step(),
                      pointerTo3DArray.xsize(),
                      pointerTo3DArray.ysize(),
                      cudaMemcpyDefault);

         currentIndex = (currentIndex + 1) % layers;
         return latestGlobalHeightMap;
      }

      int error;

      GpuMat result = new GpuMat(rows, cols, opencv_core.CV_16UC1);

      kernel.withPointer(pointerTo3DArray.ptr()).withLong(pointerTo3DArray.pitch());
      kernel.withPointer(result.data()).withLong(result.step());
      kernel.withLong(pointerTo3DArray.pitch() * rows);
      kernel.withInt(rows);
      kernel.withInt(cols);
      kernel.withInt(layers);

      dim3 gridDim = new dim3();
      dim3 blockDim = new dim3(cols, rows, 1);
      kernel.run(stream, gridDim, blockDim, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Print the result to make sure we get what we expect
      // In this example we go through 2,4,6,8 so we expect the average to be 5
      Mat cpuResult = new Mat();
      result.download(cpuResult);
      PerceptionDebugTools.printMat("Result", cpuResult, 1);

      cudaMemcpy2D(pointerTo3DArray.ptr().position(currentIndex * pointerTo3DArray.pitch() * pointerTo3DArray.ysize()),
                   pointerTo3DArray.pitch(),
                   latestGlobalHeightMap.data(),
                   latestGlobalHeightMap.step(),
                   pointerTo3DArray.xsize(),
                   pointerTo3DArray.ysize(),
                   cudaMemcpyDefault);

      currentIndex = (currentIndex + 1) % layers;

      return result;
   }

   public void reset()
   {
      loopTracker = 0;
   }

   public void close()
   {
      program.close();
      kernel.close();
   }
}
