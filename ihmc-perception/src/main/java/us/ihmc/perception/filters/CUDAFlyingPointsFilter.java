package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;

import java.net.URL;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class CUDAFlyingPointsFilter
{
   private static Mat hostOutputImage = new Mat();
   private static GpuMat deviceInputImage;
   private static GpuMat deviceOutputImage;
   private CUDAKernel flyingPointFilterKernel;
   private CUDAProgram flyingPointFilterCUDAProgram;
   private dim3 blockSize;
   private dim3 gridSize;
   private CUstream_st stream;

   public CUDAFlyingPointsFilter()
   {
      stream = CUDAStreamManager.getStream();
      cudart.cudaStreamCreate(stream);
      URL kernelPath = getClass().getResource("/CUDA/flyingPointFilterKernel.cu");
      flyingPointFilterCUDAProgram = new CUDAProgram(kernelPath);
      String filterKernelName = "filterFlyingPoints";
      flyingPointFilterKernel = flyingPointFilterCUDAProgram.loadKernel(filterKernelName);
   }

   public GpuMat setInputMat(Mat inputImage)
   {
      deviceInputImage = new GpuMat(inputImage.rows(), inputImage.cols(), opencv_core.CV_16UC1);
      deviceInputImage.upload(inputImage);
      return deviceInputImage;
   }

   public void destroy()
   {
      flyingPointFilterCUDAProgram.close();
      flyingPointFilterKernel.close();
      blockSize.close();
      gridSize.close();
      deviceInputImage.close();
      deviceOutputImage.close();
      CUDAStreamManager.releaseStream(stream);
   }

   public void applyFilter(GpuMat inputImage)
   {
      deviceOutputImage = new GpuMat(inputImage.rows(), inputImage.cols(), opencv_core.CV_16UC1);
      int blockSizeXY = 16;
      int gridSizeX = (inputImage.cols() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeY = (inputImage.rows() + blockSizeXY - 1) / blockSizeXY;
      blockSize = new dim3(blockSizeXY, blockSizeXY, 1);
      gridSize = new dim3(gridSizeX, gridSizeY, 1);
      flyingPointFilterKernel.withPointer(inputImage.data()).withLong(inputImage.step());
      flyingPointFilterKernel.withPointer(deviceOutputImage.data()).withLong(deviceOutputImage.step());
      flyingPointFilterKernel.withInt(inputImage.rows()).withInt(inputImage.cols());
      flyingPointFilterKernel.run(stream, gridSize, blockSize, 0);
      cudart.cudaStreamSynchronize(stream);
   }

   public static Mat getOutputFilteredMat()
   {
      deviceOutputImage.download(hostOutputImage);
      return hostOutputImage;
   }

   public GpuMat getOutputFilteredGpuMat()
   {
      return deviceOutputImage;
   }
}


