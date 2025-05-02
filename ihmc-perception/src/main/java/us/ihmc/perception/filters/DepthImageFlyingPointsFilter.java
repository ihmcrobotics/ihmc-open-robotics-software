package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

public class DepthImageFlyingPointsFilter
{
   private static final int BLOCK_SIZE_XY = 16;
   private final CUDAKernel flyingPointFilterKernel;
   private final CUDAProgram flyingPointFilterCUDAProgram;
   private final CUstream_st stream;

   public DepthImageFlyingPointsFilter()
   {
      stream = CUDAStreamManager.getStream();
      URL kernelPath = getClass().getResource("/us/ihmc/perception/cuda/DepthImageFlyingPointsFilter.cu");
      try
      {
         flyingPointFilterCUDAProgram = new CUDAProgram(kernelPath);
         String filterKernelName = "computeNormalsAndFilterFlyingPoints";
         flyingPointFilterKernel = flyingPointFilterCUDAProgram.loadKernel(filterKernelName);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void applyFilter(GpuMat deviceInputImage, GpuMat deviceOutputImageToPack, CameraIntrinsics cameraIntrinsics)
   {
      int gridSizeX = (deviceInputImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (deviceInputImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      flyingPointFilterKernel.withPointer(deviceInputImage.data()).withLong(deviceInputImage.step());
      flyingPointFilterKernel.withPointer(deviceOutputImageToPack.data()).withLong(deviceOutputImageToPack.step());
      flyingPointFilterKernel.withInt(deviceInputImage.cols()).withInt(deviceInputImage.rows());
      flyingPointFilterKernel.withFloat((float) cameraIntrinsics.getFx())
                             .withFloat((float) cameraIntrinsics.getFy())
                             .withFloat((float) cameraIntrinsics.getCx())
                             .withFloat((float) cameraIntrinsics.getCy());
      flyingPointFilterKernel.withFloat(0.866F).withFloat(0.05F).withFloat(0.3F).withFloat(0.02F);
      flyingPointFilterKernel.run(stream, gridSize, blockSize, 0);

      int error = cudart.cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      blockSize.close();
      gridSize.close();
   }

   public void destroy()
   {
      flyingPointFilterCUDAProgram.close();
      flyingPointFilterKernel.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
