package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class FilteredVerticalSurfacesExtractor
{
   static final int BLOCK_SIZE_XY = 32;

   private final CUstream_st stream;
   private final int rows;
   private final int cols;
   private final CUDAKernel kernel;
   private final CUDAProgram program;

   public FilteredVerticalSurfacesExtractor(CUstream_st stream, int rows, int cols)
   {
      this.stream = stream;
      this.rows = rows;
      this.cols = cols;

      // Load header and main file
      URL kernelPath = getClass().getResource("VerticalSurfacesFilter.cu");
      try
      {
         program = new CUDAProgram(kernelPath);
         kernel = program.loadKernel("removeCinderBlockWalls");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void update(GpuMat croppedHeightMap)
   {
      int error;
      GpuMat result = croppedHeightMap.clone();

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      kernel.withPointer(result.data()).withLong(result.step());
      kernel.withPointer(croppedHeightMap.data()).withLong(croppedHeightMap.step());
      kernel.withInt(rows);
      kernel.withInt(cols);
      kernel.withInt(100);

      //Execute the CUDA kernels with the provided stream
      //Each kernel performs a specific task related to the height map update, registration, and cropping
      int registerKernelGridSizeXY = (rows + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      dim3 registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

      kernel.run(stream, registerKernelGridDim, blockSize, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      cudaFree(result);
      result.close();
   }

   public void destroy()
   {
      program.close();
      kernel.close();
   }
}
