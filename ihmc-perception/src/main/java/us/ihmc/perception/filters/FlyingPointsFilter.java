package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
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

   public FlyingPointsFilter(GpuMat inputImage)
   {
      applyFilter(inputImage);
   }

   public void initialize()
   {
      stream = new CUstream_st();
      cudart.cudaStreamCreate(stream);

      // Load CUDA kernels
      URL kernelPath = getClass().getResource("RapidHeightMapExtractor.cu");
      flyingPointFilterCUDAProgram = new CUDAProgram(kernelPath);

      String updateKernelName = "heightMapUpdateKernel";
      flyingPointFilterKernel = flyingPointFilterCUDAProgram.loadKernel(updateKernelName);

   }

   public void destroy()
   {
      flyingPointFilterCUDAProgram.close();
      updateKernel.close();

      blockSize.close();
      gridSizeKernel1.close();

      cudart.cudaStreamDestroy(stream);
      stream.close();
   }

   public void applyFilter(GpuMat inputImage)
   {


      int blockSizeXY = 32;
      int gridSizeKernel0X = (inputImage.rows() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel0Y = (inputImage.cols() + blockSizeXY - 1) / blockSizeXY;

      blockSize = new dim3(blockSizeXY, blockSizeXY, 1);
      gridSizeKernel0 = new dim3(gridSizeKernel0X, gridSizeKernel0Y, 1);

      flyingPointFilterKernel.withPointer(inputDepthImage.data()).withLong(inputDepthImage.step());

      cudaStreamSynchronize(stream);

      flyingPointFilterKernel.run(stream, gridSizeKernel1, blockSize, 0);

      cudaStreamSynchronize(stream);

      Mat finalCroppedHeightMap = new Mat();  // Assuming the height map is 201x201
      sensorCroppedHeightMapImage.download(finalCroppedHeightMap);
   }
}


