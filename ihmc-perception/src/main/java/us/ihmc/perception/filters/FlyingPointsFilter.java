package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class FlyingPointsFilter
{
   private CUstream_st stream;
   private GpuMat outputFilteredImage;
   private CUDAKernel flyingPointFilterKernel;
   private CUDAProgram flyingPointFilterCUDAProgram;
   private dim3 blockSize;
   private dim3 gridSize;

   public FlyingPointsFilter(GpuMat inputImage)
   {
      initialize();
      applyFilter(inputImage);
   }

   public void initialize()
   {
      stream = new CUstream_st();
      cudart.cudaStreamCreate(stream);

      URL kernelPath = getClass().getResource("/CUDA/flyingPointFilterKernel.cu");

      flyingPointFilterCUDAProgram = new CUDAProgram(kernelPath);

      String filterKernelName = "filterFlyingPoints";
      flyingPointFilterKernel = flyingPointFilterCUDAProgram.loadKernel(filterKernelName);
   }

   public void destroy()
   {
      flyingPointFilterCUDAProgram.close();
      flyingPointFilterKernel.close();

      blockSize.close();
      gridSize.close();

      cudart.cudaStreamDestroy(stream);
      stream.close();
   }

   public void applyFilter(GpuMat inputImage)
   {

      outputFilteredImage = new GpuMat(inputImage.rows(), inputImage.cols(), opencv_core.CV_16UC1);

      int blockSizeXY = 16;
      int gridSizeX = (inputImage.rows() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeY = (inputImage.cols() + blockSizeXY - 1) / blockSizeXY;

      blockSize = new dim3(blockSizeXY, blockSizeXY, 1);
      gridSize = new dim3(gridSizeX, gridSizeY, 1);

      flyingPointFilterKernel.withPointer(inputImage.data()).withLong(inputImage.step());
      flyingPointFilterKernel.withPointer(outputFilteredImage.data()).withLong(outputFilteredImage.step());
      flyingPointFilterKernel.withInt(inputImage.rows()).withInt(inputImage.cols());

      cudaStreamSynchronize(stream);

      flyingPointFilterKernel.run(stream, gridSize, blockSize, 0);

      cudaStreamSynchronize(stream);
   }

   public GpuMat getOutputFilteredImage()
   {
      return outputFilteredImage;
   }
}


