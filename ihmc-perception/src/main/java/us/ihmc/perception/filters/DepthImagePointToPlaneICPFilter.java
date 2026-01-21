package us.ihmc.perception.filters;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class DepthImagePointToPlaneICPFilter
{
   private static final boolean PRINT_DEBUG_TIMINGS = false;
   private static final int BLOCK_SIZE_XY = 32;

   private static final int ATA_SIZE = 36;
   private static final int ATB_SIZE = 6;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   private final FloatPointer deviceATA;
   private final FloatPointer deviceATb;

   private final FloatPointer hostATA;
   private final FloatPointer hostATb;

   private float maxDepthDifference = 0.05f; // meters

   public DepthImagePointToPlaneICPFilter()
   {
      URL kernelURL = getClass().getResource("DepthImagePointToPlaneICPFilter.cu");
      URL perceptionUtilsURL = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/MathUtils.cuh");

      try
      {
         program = new CUDAProgram(kernelURL, perceptionUtilsURL, mathUtilsURL);
         kernel = program.loadKernel("pointToPlaneICP");
         kernel.enableKernelTimings(PRINT_DEBUG_TIMINGS);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      stream = CUDAStreamManager.getStream();

      deviceATA = new FloatPointer();
      deviceATb = new FloatPointer();

      CUDATools.mallocAsync(deviceATA, ATA_SIZE, stream);
      CUDATools.mallocAsync(deviceATb, ATB_SIZE, stream);

      hostATA = new FloatPointer(ATA_SIZE);
      hostATb = new FloatPointer(ATB_SIZE);
   }

   /**
    * Maximum allowed depth difference for projective correspondence.
    */
   public void setMaxDepthDifference(float maxDepthDifference)
   {
      this.maxDepthDifference = maxDepthDifference;
   }

   /**
    * Runs one ICP linearization step.
    *
    * @param sourceDepth   current frame depth image
    * @param targetDepth   previous frame or map depth image
    * @param targetNormals normal map associated with target depth
    */
   public void process(GpuMat sourceDepth, GpuMat targetDepth, GpuMat targetNormals, CameraIntrinsics intrinsics)
   {
      int error;

      // Zero accumulators
      cudaMemsetAsync(deviceATA, 0, ATA_SIZE * Float.BYTES, stream);
      cudaMemsetAsync(deviceATb, 0, ATB_SIZE * Float.BYTES, stream);

      int gridSizeX = (sourceDepth.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (sourceDepth.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;

      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      kernel.withPointer(sourceDepth.data());
      kernel.withLong(sourceDepth.step());

      kernel.withPointer(targetDepth.data());
      kernel.withLong(targetDepth.step());

      kernel.withPointer(targetNormals.data());

      kernel.withInt(sourceDepth.cols());
      kernel.withInt(sourceDepth.rows());

      kernel.withFloat((float) intrinsics.getFx());
      kernel.withFloat((float) intrinsics.getFy());
      kernel.withFloat((float) intrinsics.getCx());
      kernel.withFloat((float) intrinsics.getCy());

      kernel.withFloat(maxDepthDifference);

      kernel.withPointer(deviceATA);
      kernel.withPointer(deviceATb);

      kernel.run(stream, gridSize, blockSize, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      blockSize.close();
      gridSize.close();
   }

   /**
    * Copies the accumulated normal equations back to host memory.
    */
   public void downloadResults()
   {
      CUDATools.memcpyAsync(hostATA, deviceATA, ATA_SIZE, stream);
      CUDATools.memcpyAsync(hostATb, deviceATb, ATB_SIZE, stream);
   }

   public FloatPointer getATA()
   {
      return hostATA;
   }

   public FloatPointer getATb()
   {
      return hostATb;
   }

   public void close()
   {
      cudaFree(deviceATA);
      cudaFree(deviceATb);

      deviceATA.close();
      deviceATb.close();
      hostATA.close();
      hostATb.close();

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
