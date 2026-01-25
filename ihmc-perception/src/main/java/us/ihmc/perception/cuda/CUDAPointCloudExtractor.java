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

      // Allocate fixed size page-locked memory (on host)
      error = cudaMallocHost(transformPointer, 16L * transformPointer.sizeof()); // 16 floats for transform matrix
      CUDATools.checkCUDAError(error);
      error = cudaMallocHost(pointCloudSize, pointCloudSize.sizeof());           // 1 int for point cloud size
      CUDATools.checkCUDAError(error);
   }

   /**
    * Extract a point cloud from a depth image.
    *
    * @param depthImage A 16 bit depth image.
    * @return The point cloud extracted from the depth image, as a list of {@link Point3D32}s.
    * @apiNote Points at 0 depth are ignored, as it usually means no depth value was assigned to the pixel.
    *       Thus, the number of points in the output will not necessarily equal the number of pixels within the depth image.
    */
   public List<Point3D32> extractPointCloud(RawImage depthImage)
   {
      if (depthImage.get() == null)
         return new ArrayList<>();

      // Ensure enough GPU memory is allocated
      long maxPointCloudFloats = 3L * depthImage.getWidth() * depthImage.getHeight();
      if (gpuPointCloudPointer.isNull() || gpuPointCloudPointer.limit() < maxPointCloudFloats)
      {
         if (!gpuPointCloudPointer.isNull())
         {  // de-allocate existing allocation
            error = cudaFreeAsync(gpuPointCloudPointer, stream);
            CUDATools.checkCUDAError(error);
         }
         CUDATools.mallocAsync(gpuPointCloudPointer, maxPointCloudFloats, stream); // allocate enough memory
         gpuPointCloudPointer.limit(maxPointCloudFloats);
      }

      // Get a float[] of the transformation matrix, and copy it into CUDA page-locked memory
      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);

      // Calculate block size and grid size of the kernel launch
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      // Run the kernel
      kernel.withPointer(depthImage.getCUDADataPointer())
            .withLong(depthImage.getGpuImageMat().step())
            .withInt(depthImage.getWidth())
            .withInt(depthImage.getHeight())
            .withFloat(depthImage.getFocalLengthX())
            .withFloat(depthImage.getFocalLengthY())
            .withFloat(depthImage.getPrincipalPointX())
            .withFloat(depthImage.getPrincipalPointY())
            .withFloat(depthImage.getDepthDiscretization())
            .withPointer(transformPointer)
            .withPointer(gpuPointCloudPointer)
            .withPointer(pointCloudSize)
            .run(stream, gridSize, blockSize, 0);

      // Synchronize the stream to ensure we can read the point cloud size
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
      int numberOfPoints = pointCloudSize.get();
      long numberOfFloats = 3L * numberOfPoints;

      // Copy the point cloud data from GPU to CPU, then to a Java float[]
      FloatPointer cpuPointCloudPointer = new FloatPointer(numberOfFloats);
      float[] pointsArray = new float[(int) numberOfFloats];
      CUDATools.memcpyAsync(cpuPointCloudPointer, gpuPointCloudPointer, numberOfFloats, stream);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
      cpuPointCloudPointer.get(pointsArray);

      // Create a list of points from the float[]
      Point3D32[] result = new Point3D32[numberOfPoints];
      IntStream.range(0, numberOfPoints).parallel().forEach(i ->
      {
         float x = pointsArray[3 * i];
         float y = pointsArray[3 * i + 1];
         float z = pointsArray[3 * i + 2];
         result[i] = new Point3D32(x, y, z);
      });

      // Release stuff
      blockSize.close();
      gridSize.close();
      cpuPointCloudPointer.close();
      depthImage.release();

      return Arrays.asList(result);
   }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeHost(transformPointer));
      CUDATools.checkCUDAError(cudaFreeHost(pointCloudSize));
      if (!gpuPointCloudPointer.isNull())
         CUDATools.checkCUDAError(cudaFreeAsync(gpuPointCloudPointer, stream));

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
