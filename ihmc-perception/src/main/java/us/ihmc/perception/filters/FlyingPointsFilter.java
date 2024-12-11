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

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class FlyingPointsFilter
{
   private CUstream_st stream;
   private Mat hostOutputImage = new Mat();
   private GpuMat deviceInputImage;
   private GpuMat deviceOutputImage;
   private CUDAKernel flyingPointFilterKernel;
   private CUDAProgram flyingPointFilterCUDAProgram;
   private dim3 blockSize;
   private dim3 gridSize;

   public FlyingPointsFilter(Mat inputImage)
   {
      initialize();
      deviceInputImage = new GpuMat(inputImage.rows(), inputImage.cols(), opencv_core.CV_16UC1);
      deviceOutputImage = new GpuMat(inputImage.rows(), inputImage.cols(), opencv_core.CV_16UC1);
      deviceInputImage.upload(inputImage);
      applyFilter(deviceInputImage);
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

      deviceInputImage.close();
      deviceOutputImage.close();
   }

   public void applyFilter(GpuMat inputImage)
   {
      int blockSizeXY = 16;
      int gridSizeX = (inputImage.cols() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeY = (inputImage.rows() + blockSizeXY - 1) / blockSizeXY;

      blockSize = new dim3(blockSizeXY, blockSizeXY, 1);
      gridSize = new dim3(gridSizeX, gridSizeY, 1);

      flyingPointFilterKernel.withPointer(inputImage.data()).withLong(inputImage.step());
      flyingPointFilterKernel.withPointer(deviceOutputImage.data()).withLong(deviceOutputImage.step());
      flyingPointFilterKernel.withInt(inputImage.rows()).withInt(inputImage.cols());

      cudaStreamSynchronize(stream);

      flyingPointFilterKernel.run(stream, gridSize, blockSize, 0);

      cudaStreamSynchronize(stream);
   }

   public Mat getOutputFilteredMat()
   {

      Mat localMat = new Mat();
      deviceInputImage.download(localMat);
      Mat globalMat = new Mat();
      deviceOutputImage.download(globalMat);

      PerceptionDebugTools.display("Input Height Map", localMat, 1);
      PerceptionDebugTools.display(" Transfomed Input Height Map", globalMat, 1);

      deviceOutputImage.download(hostOutputImage);
      destroy();

      return hostOutputImage;
   }
}


