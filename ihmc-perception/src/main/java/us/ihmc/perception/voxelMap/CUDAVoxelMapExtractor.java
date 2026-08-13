package us.ihmc.perception.voxelMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Fuses depth images into a robot-centric voxel occupancy map on the GPU.
 */
public class CUDAVoxelMapExtractor implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   // Number of voxels in X, Y, and Z dimensions
   private final int mapSizeX;
   private final int mapSizeY;
   private final int mapSizeZ;

   // Size of each voxel (e.g. 0.1 = 10cm cube)
   private final float voxelSize;

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel fuseDepthImageKernel;
   private final CUstream_st stream;

   // Pre-allocated useful objects
   private final RigidBodyTransform worldToMapTransform = new RigidBodyTransform();
   private final RigidBodyTransform depthToMapTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];

   // Page-locked host memory holding each depth image's depth-to-map transform. One buffer per image:
   // the kernels read these while executing asynchronously, so reusing a single buffer would let the
   // host-side write for image i + 1 race the device-side reads of image i's still-running kernel.
   private final List<FloatPointer> transformPointers = new ArrayList<>();

   private int error;

   public CUDAVoxelMapExtractor(int mapSize, float voxelSize)
   {
      this(mapSize, mapSize, mapSize, voxelSize);
   }

   public CUDAVoxelMapExtractor(int mapSizeX, int mapSizeY, int mapSizeZ, float voxelSize)
   {
      this.mapSizeX = mapSizeX;
      this.mapSizeY = mapSizeY;
      this.mapSizeZ = mapSizeZ;
      this.voxelSize = voxelSize;

      // Get URLs to the CUDA files
      URL voxelMapExtractionURL = CUDATools.class.getResource("VoxelMapExtraction.cu");
      URL utilsURL = CUDATools.getUtilsFile();
      URL perceptionUtilsURL = CUDATools.getPerceptionUtilsFile();
      URL mathUtilsURL = CUDATools.class.getResource("MathUtils.cuh");

      // Compile the program and get the kernel
      try
      {
         program = new CUDAProgram(voxelMapExtractionURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         fuseDepthImageKernel = program.loadKernel("fuseDepthImageIntoVoxelMap");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // Get a stream
      stream = CUDAStreamManager.getStream();
   }

   /**
    * Fuses the given depth images into a voxel occupancy map around the given origin.
    * <p>
    * Each valid depth pixel is unprojected into 3D, transformed into the map frame defined by
    * {@code origin}, and quantized to its nearest voxel using center-anchored indexing
    * ({@code i = round(local / voxelSize + (N-1)/2)}). Pixels with 0 depth (no measurement)
    * and points landing outside the map are silently dropped.
    *
    * @param origin      pose of the map center in world frame (world_T_map)
    * @param depthImages 16-bit depth images to fuse
    * @return a {@link VoxelMap} whose GPU data pointer holds the freshly computed occupancy values;
    *       CPU data is fetched lazily on first call to {@link VoxelMap#getCpuData()}
    */
   public VoxelMap getVoxelMap(Pose3D origin, RawImage... depthImages)
   {
      // Allocate device memory for the voxel map
      int voxelCount = getNumberOfVoxels();
      FloatPointer voxelMapPointer = new FloatPointer();
      CUDATools.mallocAsync(voxelMapPointer, voxelCount, stream);
      voxelMapPointer.limit(voxelCount);

      origin.get(worldToMapTransform);
      worldToMapTransform.invert();

      int imageIndex = 0;
      for (RawImage depthImage : depthImages)
      {
         if (depthImage == null || depthImage.get() == null)
            continue;

         // Compose depth frame -> world -> map frame into a single transform on the CPU,
         // and copy it into this image's page-locked buffer
         depthToMapTransform.set(worldToMapTransform);
         depthToMapTransform.multiply(depthImage.getTransformToWorld());
         depthToMapTransform.get(transformArray);
         FloatPointer transformPointer = getTransformPointer(imageIndex++);
         transformPointer.put(transformArray);

         // Calculate block size and grid size of the kernel launch
         try (dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
              dim3 gridSize = new dim3(Math.max(1, (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2)),
                                       Math.max(1, (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2)),
                                       1))
         {
            // Run the kernel
            fuseDepthImageKernel.withPointer(depthImage.getCUDADataPointer())
                                .withLong(depthImage.getGpuImageMat().step())
                                .withInt(depthImage.getWidth())
                                .withInt(depthImage.getHeight())
                                .withFloat(depthImage.getFocalLengthX())
                                .withFloat(depthImage.getFocalLengthY())
                                .withFloat(depthImage.getPrincipalPointX())
                                .withFloat(depthImage.getPrincipalPointY())
                                .withFloat(depthImage.getDepthDiscretization())
                                .withPointer(transformPointer)
                                .withInt(mapSizeX)
                                .withInt(mapSizeY)
                                .withInt(mapSizeZ)
                                .withFloat(voxelSize)
                                .withPointer(voxelMapPointer)
                                .run(stream, gridSize, blockSize, 0);
         }

         depthImage.release();
      }

      // Synchronize so the returned map is fully built and safe to read from any stream
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      return new VoxelMap(null, voxelMapPointer, mapSizeX, mapSizeY, mapSizeZ, voxelSize, origin);
   }

   public int getNumberOfVoxels()
   {
      return mapSizeX * mapSizeY * mapSizeZ;
   }

   private FloatPointer getTransformPointer(int imageIndex)
   {
      while (transformPointers.size() <= imageIndex)
      {
         FloatPointer transformPointer = new FloatPointer();
         error = cudaMallocHost(transformPointer, 16L * transformPointer.sizeof()); // 16 floats for transform matrix
         CUDATools.checkCUDAError(error);
         transformPointers.add(transformPointer);
      }

      return transformPointers.get(imageIndex);
   }

   @Override
   public void close()
   {
      for (FloatPointer transformPointer : transformPointers)
         CUDATools.checkCUDAError(cudaFreeHost(transformPointer));

      fuseDepthImageKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
