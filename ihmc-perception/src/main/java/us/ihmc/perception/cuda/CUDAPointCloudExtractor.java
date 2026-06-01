package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Extracts 3D point clouds from depth images.
 */
public class CUDAPointCloudExtractor implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   // Pre-allocated useful objects
   private final RigidBodyTransform depthToWorldTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];

   // Pointers to CUDA memory
   private final FloatPointer transformPointer = new FloatPointer();
   private final FloatPointer gpuPointCloudPointer = new FloatPointer();
   private final IntPointer pointCloudSize = new IntPointer();

   private int error;

   public CUDAPointCloudExtractor()
   {
      // Get URLs to the CUDA files
      URL pointCloudExtractionURL = getClass().getResource("PointCloudExtraction.cu");
      URL utilsURL = getClass().getResource("Utils.cu");
      URL perceptionUtilsURL = getClass().getResource("PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      // Compile the program, and get the kernel
      try
      {
         program = new CUDAProgram(pointCloudExtractionURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         kernel = program.loadKernel("extractPointCloud");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // Get a stream
      stream = CUDAStreamManager.getStream();

      CUDATools.mallocHost(transformPointer, 16L);
      CUDATools.mallocHost(pointCloudSize, 1L);
   }

   /**
    * Extract a point cloud from a depth image directly into GPU memory — no CPU round-trip.
    *
    * <p>The result stays in the GPU buffer returned by {@link #getGpuPointCloud()}.
    * Call {@link #getStream()} and synchronize before passing the buffer to another GPU consumer.
    *
    * @param depthImage    16-bit depth image. Released on return.
    * @param minDepthMeters Points closer than this are discarded (dead band / self-collision filter).
    * @param maxDepthMeters Points farther than this are discarded.
    * @return Number of valid points extracted (0 when the image is empty).
    */
   public int extractToGpu(RawImage depthImage, float minDepthMeters, float maxDepthMeters)
   {
      if (depthImage.get() == null)
      {
         depthImage.release();
         return 0;
      }

      long maxPointCloudFloats = 3L * depthImage.getWidth() * depthImage.getHeight();
      if (gpuPointCloudPointer.isNull() || gpuPointCloudPointer.limit() < maxPointCloudFloats)
      {
         if (!gpuPointCloudPointer.isNull())
            CUDATools.checkCUDAError(cudaFreeAsync(gpuPointCloudPointer, stream));
         CUDATools.mallocAsync(gpuPointCloudPointer, maxPointCloudFloats, stream);
         gpuPointCloudPointer.limit(maxPointCloudFloats);
      }

      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);
      pointCloudSize.put(0);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      kernel.withPointer(depthImage.getCUDADataPointer())
            .withLong(depthImage.getGpuImageMat().step())
            .withInt(depthImage.getWidth())
            .withInt(depthImage.getHeight())
            .withFloat(depthImage.getFocalLengthX())
            .withFloat(depthImage.getFocalLengthY())
            .withFloat(depthImage.getPrincipalPointX())
            .withFloat(depthImage.getPrincipalPointY())
            .withFloat(depthImage.getDepthDiscretization())
            .withFloat(minDepthMeters)
            .withFloat(maxDepthMeters)
            .withPointer(transformPointer)
            .withPointer(gpuPointCloudPointer)
            .withPointer(pointCloudSize)
            .run(stream, gridSize, blockSize, 0);

      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      int count = pointCloudSize.get();

      blockSize.close();
      gridSize.close();
      depthImage.release();

      return count;
   }

   /** GPU float3 buffer holding the result of the last {@link #extractToGpu} call. */
   public FloatPointer getGpuPointCloud()
   {
      return gpuPointCloudPointer;
   }

   /** CUDA stream used by this extractor. Synchronize before consuming {@link #getGpuPointCloud()} on another stream. */
   public CUstream_st getStream()
   {
      return stream;
   }

   @Override
   public void close()
   {
      CUDATools.freeHost(transformPointer);
      CUDATools.freeHost(pointCloudSize);
      if (!gpuPointCloudPointer.isNull())
         CUDATools.checkCUDAError(cudaFreeAsync(gpuPointCloudPointer, stream));

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
