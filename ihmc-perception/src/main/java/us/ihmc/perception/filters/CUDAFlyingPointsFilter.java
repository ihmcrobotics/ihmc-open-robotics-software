package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;

import java.net.URL;

import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

public class CUDAFlyingPointsFilter
{
   private static final int BLOCK_SIZE_XY = 16;
   
   private final CUDAKernel flyingPointFilterKernel;
   private final CUDAProgram flyingPointFilterCUDAProgram;
   private final CUstream_st stream;

   public CUDAFlyingPointsFilter()
   {
      stream = CUDAStreamManager.getStream();
      URL kernelPath = getClass().getResource("/CUDA/flyingPointFilterKernel.cu");
      flyingPointFilterCUDAProgram = new CUDAProgram(kernelPath);
      String filterKernelName = "filterFlyingPoints";
      flyingPointFilterKernel = flyingPointFilterCUDAProgram.loadKernel(filterKernelName);
   }
   
   public void destroy()
   {
      flyingPointFilterCUDAProgram.close();
      flyingPointFilterKernel.close();
      CUDAStreamManager.releaseStream(stream);
   }

   public GpuMat applyFilter(GpuMat inputImage)
   {
      GpuMat deviceOutputImage = new GpuMat(inputImage.rows(), inputImage.cols(), opencv_core.CV_16UC1);

      int gridSizeX = (inputImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (inputImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      flyingPointFilterKernel.withPointer(inputImage.data()).withLong(inputImage.step());
      flyingPointFilterKernel.withPointer(deviceOutputImage.data()).withLong(deviceOutputImage.step());
      flyingPointFilterKernel.withInt(inputImage.rows()).withInt(inputImage.cols());
      flyingPointFilterKernel.run(stream, gridSize, blockSize, 0);
      CUDATools.checkCUDAError(cudart.cudaStreamSynchronize(stream));

      blockSize.close();
      gridSize.close();

      return deviceOutputImage;
   }
}
