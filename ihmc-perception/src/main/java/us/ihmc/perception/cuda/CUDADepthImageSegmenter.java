package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Sets non-mask pixels of a depth image to zero, removing the background.
 */
public class CUDADepthImageSegmenter implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   private final RigidBodyTransform depthToMaskTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];
   private final FloatPointer transformPointer = new FloatPointer();

   private final dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
   private final dim3 gridSize = new dim3();

   private int error;

   public CUDADepthImageSegmenter() throws Exception
   {
      if (!CUDATools.hasCUDADevice())
         throw new Exception("CUDA unavailable.");

      // Get the URLs to all the cu files
      URL segmentation = getClass().getResource("DepthImageSegmentation.cu");
      URL utils = getClass().getResource("Utils.cu");
      URL perceptionUtils = getClass().getResource("PerceptionUtils.cu");
      URL mathUtils = getClass().getResource("MathUtils.cuh");

      // Compile the program and get the kernel
      program = new CUDAProgram(segmentation, utils, perceptionUtils, mathUtils);
      kernel = program.loadKernel("segmentDepthImage");

      // Get a stream
      stream = CUDAStreamManager.getStream();

      // Allocate fixed size page-locked memory (on host)
      error = cudaMallocHost(transformPointer, 16L * transformPointer.sizeof()); // 16 floats for transform matrix
      CUDATools.throwCUDAError(error);
   }

   public RawImage removeBackground(RawImage depthImage, RawImage mask)
   {
      // Ensure we get the images
      if (depthImage.get() == null)
         return null;

      if (mask.get() == null)
      {
         depthImage.release();
         return null;
      }

      // Update the transform array
      depthImage.getTransformToWorld().inverseTransform(mask.getTransformToWorld(), depthToMaskTransform);
      depthToMaskTransform.get(transformArray);
      transformPointer.put(transformArray);

      // Get the GPU mats
      GpuMat depthMat = depthImage.getGpuImageMat();
      GpuMat maskMat = mask.getGpuImageMat();
      GpuMat outputMat = new GpuMat(depthMat.size(), depthMat.type());

      // Update the necessary grid size
      gridSize.x((depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2));
      gridSize.y((depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2));

      // Run the kernel
      kernel.withPointer(depthMat.data()).withLong(depthMat.step())
            .withInt(depthImage.getWidth()).withInt(depthImage.getHeight())
            .withFloat(depthImage.getFocalLengthX()).withFloat(depthImage.getFocalLengthY())
            .withFloat(depthImage.getPrincipalPointX()).withFloat(depthImage.getPrincipalPointY())
            .withPointer(maskMat.data()).withLong(maskMat.step())
            .withInt(mask.getWidth()).withInt(mask.getHeight())
            .withFloat(mask.getFocalLengthX()).withFloat(mask.getFocalLengthY())
            .withFloat(mask.getPrincipalPointX()).withFloat(mask.getPrincipalPointY())
            .withFloat(depthImage.getDepthDiscretization())
            .withPointer(transformPointer)
            .withPointer(outputMat.data()).withLong(outputMat.step())
            .run(stream, gridSize, blockSize, 0);

      // Synchronize to ensure the output data is contained in the output mat
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Get the result as a RawImage
      RawImage result = depthImage.replaceImage(outputMat);

      // Release the RawImages
      depthImage.release();
      mask.release();

      // Return the result
      return result;
   }

   @Override
   public void close()
   {
      error = cudaFreeHost(transformPointer);
      CUDATools.checkCUDAError(error);

      blockSize.close();
      gridSize.close();

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
