package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;

import java.net.URL;

import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

public class DepthImageFlyingPointsFilter
{
   private static final boolean PRINT_DEBUGGING_TIMINGS = false;
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAKernel flyingPointFilterKernel;
   private final CUDAProgram flyingPointFilterCUDAProgram;
   private final CUstream_st stream;

   private final DepthImageFilteringParameters depthImageFilteringParameters;

   /**
    * This runs a CUDA kernel that will remove the flying points from a depth map.
    */
   public DepthImageFlyingPointsFilter(DepthImageFilteringParameters depthImageFilteringParameters)
   {
      this.depthImageFilteringParameters = depthImageFilteringParameters;
      stream = CUDAStreamManager.getStream();
      URL kernelPath = getClass().getResource("/us/ihmc/perception/cuda/DepthImageFlyingPointsFilter.cu");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      try
      {
         flyingPointFilterCUDAProgram = new CUDAProgram(kernelPath, mathUtilsHeaderPath);
         String filterKernelName = "filterFlyingPoints";
         flyingPointFilterKernel = flyingPointFilterCUDAProgram.loadKernel(filterKernelName);
         flyingPointFilterKernel.enableKernelTimings(PRINT_DEBUGGING_TIMINGS);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * This method applies a flying points filter on the deviceInputImage.
    * The result is packed into the deviceOutputImageToPack.
    * Closing of these GpuMat's happens outside of this method.
    * To avoid memory leaks, the method returns void and doesn't allocate any memory for the GpuMat's
    *
    * @param deviceInputImage        the depth map that the flying points filter will be applied too
    * @param deviceOutputImageToPack the depth map that the result will be packed into
    */
   public void applyFilter(GpuMat deviceInputImage, GpuMat deviceOutputImageToPack, CameraIntrinsics cameraIntrinsics)
   {
      int gridSizeX = (deviceInputImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (deviceInputImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      flyingPointFilterKernel.withPointer(deviceInputImage.data()).withLong(deviceInputImage.step());
      flyingPointFilterKernel.withPointer(deviceOutputImageToPack.data()).withLong(deviceOutputImageToPack.step());
      flyingPointFilterKernel.withInt(deviceInputImage.rows()).withInt(deviceInputImage.cols());
      flyingPointFilterKernel.withInt(depthImageFilteringParameters.getWindowSizeInPixels());
      flyingPointFilterKernel.withInt(depthImageFilteringParameters.getRANSACIterations());
      flyingPointFilterKernel.withInt(depthImageFilteringParameters.getMinimumDepthValuesRequiredInWindow());
      flyingPointFilterKernel.withFloat((float) depthImageFilteringParameters.getAngleThresholdInRadians());
      flyingPointFilterKernel.withFloat((float) depthImageFilteringParameters.getNormalAngleThreshold());
      flyingPointFilterKernel.withFloat((float) cameraIntrinsics.getFx())
                             .withFloat((float) cameraIntrinsics.getFy())
                             .withFloat((float) cameraIntrinsics.getCx())
                             .withFloat((float) cameraIntrinsics.getCy());

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
