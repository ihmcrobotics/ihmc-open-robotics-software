package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;

import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Extracts 3D point clouds from depth images.
 */
public class CUDAPointCloudExtractor implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   /** Null device pointer passed for the confidence image when none is supplied. */
   private static final FloatPointer NULL_POINTER = new FloatPointer();

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
   private final FloatPointer gpuPointConfidencePointer = new FloatPointer();
   private final IntPointer pointCloudSize = new IntPointer();

   // Distance-dependent confidence model (see depthConfidenceWeight in PerceptionUtils.cu).
   // Measurements at confidenceReferenceRange get full weight; farther points are weighted
   // down by (referenceRange / range) ^ falloffExponent, then scaled by cameraTrustWeight.
   private float confidenceReferenceRange   = 2.0f;
   private float confidenceFalloffExponent  = 2.0f;
   private float cameraTrustWeight          = 1.0f;

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
      return extractToGpu(depthImage, null, minDepthMeters, maxDepthMeters);
   }

   /**
    * Same as {@link #extractToGpu(RawImage, float, float)}, but additionally folds the ZED per-pixel
    * depth-confidence map into each point's weight (returned by {@link #getGpuPointConfidence()}).
    *
    * @param depthImage      16-bit depth image. Released on return.
    * @param confidenceImage Per-pixel F32 confidence map (0/1 = best ... 100 = unreliable), aligned with
    *                        {@code depthImage}; may be {@code null} to weight by range only. Released on return.
    * @param minDepthMeters  Points closer than this are discarded (dead band / self-collision filter).
    * @param maxDepthMeters  Points farther than this are discarded.
    * @return Number of valid points extracted (0 when the depth image is empty).
    */
   public int extractToGpu(RawImage depthImage, RawImage confidenceImage, float minDepthMeters, float maxDepthMeters)
   {
      if (depthImage.get() == null)
      {
         depthImage.release();
         if (confidenceImage != null)
            confidenceImage.release();
         return 0;
      }

      // Acquire the confidence image if one was provided and is still alive.
      boolean hasConfidence = false;
      if (confidenceImage != null)
      {
         if (confidenceImage.get() != null)
            hasConfidence = true;
         else
            confidenceImage.release();
      }

      long maxPoints = (long) depthImage.getWidth() * depthImage.getHeight();
      long maxPointCloudFloats = 3L * maxPoints;
      if (gpuPointCloudPointer.isNull() || gpuPointCloudPointer.limit() < maxPointCloudFloats)
      {
         if (!gpuPointCloudPointer.isNull())
            CUDATools.checkCUDAError(cudaFreeAsync(gpuPointCloudPointer, stream));
         CUDATools.mallocAsync(gpuPointCloudPointer, maxPointCloudFloats, stream);
         gpuPointCloudPointer.limit(maxPointCloudFloats);
      }
      if (gpuPointConfidencePointer.isNull() || gpuPointConfidencePointer.limit() < maxPoints)
      {
         if (!gpuPointConfidencePointer.isNull())
            CUDATools.checkCUDAError(cudaFreeAsync(gpuPointConfidencePointer, stream));
         CUDATools.mallocAsync(gpuPointConfidencePointer, maxPoints, stream);
         gpuPointConfidencePointer.limit(maxPoints);
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
            .withPointer(gpuPointConfidencePointer)
            .withFloat(confidenceReferenceRange)
            .withFloat(confidenceFalloffExponent)
            .withFloat(cameraTrustWeight)
            .withPointer(hasConfidence ? confidenceImage.getCUDADataPointer() : NULL_POINTER)
            .withLong(hasConfidence ? confidenceImage.getGpuImageMat().step() : 0L)
            .withInt(hasConfidence ? 1 : 0)
            .withPointer(pointCloudSize)
            .run(stream, gridSize, blockSize, 0);

      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      int count = pointCloudSize.get();

      blockSize.close();
      gridSize.close();
      depthImage.release();
      if (hasConfidence)
         confidenceImage.release();

      return count;
   }

   /**
    * Extracts a world-frame point cloud from a depth image and returns it on the CPU as a
    * list of {@link Point3D32}s (no range filtering). Convenience wrapper over
    * {@link #extractToGpu(RawImage, float, float)} that downloads the GPU buffer.
    *
    * @param depthImage 16-bit depth image. Released on return.
    * @return The extracted points, or an empty list when the image is empty.
    */
   public List<Point3D32> extractPointCloud(RawImage depthImage)
   {
      int numberOfPoints = extractToGpu(depthImage, 0.0f, Float.MAX_VALUE);
      if (numberOfPoints <= 0)
         return new ArrayList<>();

      // Copy the point cloud data from GPU to CPU, then to a Java float[]
      long numberOfFloats = 3L * numberOfPoints;
      FloatPointer cpuPointCloudPointer = new FloatPointer(numberOfFloats);
      float[] pointsArray = new float[(int) numberOfFloats];
      CUDATools.memcpyAsync(cpuPointCloudPointer, gpuPointCloudPointer, numberOfFloats, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      cpuPointCloudPointer.get(pointsArray);
      cpuPointCloudPointer.close();

      // Create a list of points from the float[]
      Point3D32[] result = new Point3D32[numberOfPoints];
      IntStream.range(0, numberOfPoints).parallel().forEach(i ->
      {
         float x = pointsArray[3 * i];
         float y = pointsArray[3 * i + 1];
         float z = pointsArray[3 * i + 2];
         result[i] = new Point3D32(x, y, z);
      });

      return Arrays.asList(result);
   }

   /** GPU float3 buffer holding the result of the last {@link #extractToGpu} call. */
   public FloatPointer getGpuPointCloud()
   {
      return gpuPointCloudPointer;
   }

   /**
    * GPU float buffer of per-point confidence weights in (0, 1], parallel to
    * {@link #getGpuPointCloud()}, from the last {@link #extractToGpu} call. Pass alongside the
    * point cloud to {@link CUDAGPUVoxelGrid#updateHits(FloatPointer, FloatPointer, int)} so
    * distant, higher-variance measurements contribute less to occupancy.
    */
   public FloatPointer getGpuPointConfidence()
   {
      return gpuPointConfidencePointer;
   }

   /**
    * Configures the distance-dependent measurement-confidence model.
    *
    * @param referenceRange  Range (m) at which a measurement gets full weight; nearer points are capped at 1.
    * @param falloffExponent (referenceRange / range) exponent: 4 ≈ strict inverse-variance (sigma ∝ z²), 2 is gentler.
    * @param cameraTrust     Per-camera scalar multiplying the whole curve, to weight cameras differently.
    */
   public void setConfidenceModel(float referenceRange, float falloffExponent, float cameraTrust)
   {
      this.confidenceReferenceRange  = referenceRange;
      this.confidenceFalloffExponent = falloffExponent;
      this.cameraTrustWeight         = cameraTrust;
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
      if (!gpuPointConfidencePointer.isNull())
         CUDATools.checkCUDAError(cudaFreeAsync(gpuPointConfidencePointer, stream));

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
