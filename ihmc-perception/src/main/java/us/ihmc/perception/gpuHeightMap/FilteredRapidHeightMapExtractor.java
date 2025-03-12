package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.cudaExtent;
import org.bytedeco.cuda.cudart.cudaPitchedPtr;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class FilteredRapidHeightMapExtractor
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;

   private static final int BLOCK_SIZE_XY = 32;
   private static final float ALPHA = 0.75F;

   private final cudaPitchedPtr pointerTo3DArray;
   private int currentIndex;
   private final int layers;

   private final CUstream_st stream;
   private final CUDAKernel kernel;
   private final CUDAProgram program;
   private int loopTracker = 0;

   public FilteredRapidHeightMapExtractor(CUstream_st stream, int rows, int cols, int layers)
   {
      this.stream = stream;
      this.layers = layers;

      // Load header and main file
      URL kernelPath = getClass().getResource("FilteredRapidHeightMapExtractor.cu");
      try
      {
         program = new CUDAProgram(kernelPath);
         kernel = program.loadKernel("filterRapidHeightMap");
         kernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
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
   }

   public GpuMat update(GpuMat latestGlobalHeightMap)
   {
      int error;

      // Only want to compute the average if we have the past values to use
      if (loopTracker < layers)
      {
         loopTracker++;

         error = cudaMemcpy2D(pointerTo3DArray.ptr().position(currentIndex * pointerTo3DArray.pitch() * pointerTo3DArray.ysize()),
                              pointerTo3DArray.pitch(),
                              latestGlobalHeightMap.data(),
                              latestGlobalHeightMap.step(),
                              pointerTo3DArray.xsize(),
                              pointerTo3DArray.ysize(),
                              cudaMemcpyDefault);

         CUDATools.checkCUDAError(error);

         currentIndex = (currentIndex + 1) % layers;
         return latestGlobalHeightMap.clone();
      }

      // Allocate the correct amount of threads to process the entire mat
      int registerKernelGridSizeXY = (latestGlobalHeightMap.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      dim3 registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

      // Create new GpuMat to store the result
      GpuMat result = new GpuMat(latestGlobalHeightMap.rows(), latestGlobalHeightMap.cols(), opencv_core.CV_16UC1);

      kernel.withPointer(pointerTo3DArray.ptr()).withLong(pointerTo3DArray.pitch());
      kernel.withPointer(result.data()).withLong(result.step());
      kernel.withPointer(latestGlobalHeightMap.data()).withLong(latestGlobalHeightMap.step());
      kernel.withInt(layers);
      kernel.withLong(pointerTo3DArray.pitch() * latestGlobalHeightMap.rows());
      kernel.withInt(latestGlobalHeightMap.rows());
      kernel.withInt(latestGlobalHeightMap.cols());
      kernel.withFloat(ALPHA);

      kernel.run(stream, registerKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      latestGlobalHeightMap.close();
      latestGlobalHeightMap = result.clone();
      result.close();

      // Upload the latest height map to the GPU for the next iteration
      error = cudaMemcpy2D(pointerTo3DArray.ptr().position(currentIndex * pointerTo3DArray.pitch() * pointerTo3DArray.ysize()),
                           pointerTo3DArray.pitch(),
                           latestGlobalHeightMap.data(),
                           latestGlobalHeightMap.step(),
                           pointerTo3DArray.xsize(),
                           pointerTo3DArray.ysize(),
                           cudaMemcpyDefault);

      CUDATools.checkCUDAError(error);

      currentIndex = (currentIndex + 1) % layers;

      return latestGlobalHeightMap.clone();
   }

   public void reset()
   {
      loopTracker = 0;
   }

   public void destroy()
   {
      program.close();
      kernel.close();
      int error = cudaFree(pointerTo3DArray);
      CUDATools.checkCUDAError(error);
   }
}
