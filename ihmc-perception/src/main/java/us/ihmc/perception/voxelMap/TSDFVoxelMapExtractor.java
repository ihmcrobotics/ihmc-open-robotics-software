package us.ihmc.perception.voxelMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
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
 * Fuses depth images into a robot-centric TSDF (truncated signed-distance field) voxel map on the GPU.
 *
 * Matches {@code depth_voxel_tsdf.fuse_depth_cameras_to_tsdf()} exactly:
 * <ul>
 *   <li>Voxel-centric projection: each voxel is projected into the camera's pixel space, the
 *       sampled depth is backprojected to a world surface point, and the pelvis-anchored signed
 *       distance is computed and clamped to [-1, 1].</li>
 *   <li>Multi-camera fusion via minimum (most-occluded reading wins across cameras).</li>
 *   <li>Un-observed voxels stay at +1.0 (free space).</li>
 * </ul>
 *
 * Temporal ego-motion warping and merging with a persistent memory grid is handled by the calling
 * {@link TSDFVoxelMappingThread}, matching {@code RobotCentricTSDFFromDepthTerm}'s logic.
 */
public class TSDFVoxelMapExtractor implements AutoCloseable
{
   static final float FREE_SPACE_VALUE = 1.0f;

   private static final int BLOCK_SIZE = 128;

   private final int mapSize;
   private final float voxelSize;
   private final float truncationDistance;
   private final int voxelCount;

   private final CUDAProgram program;
   private final CUDAKernel fuseDepthImageKernel;
   private final CUstream_st stream;

   // Reusable CPU-side transforms (not shared across calls)
   private final RigidBodyTransform worldToMapTransform = new RigidBodyTransform();
   private final RigidBodyTransform depthToMapTransform = new RigidBodyTransform();
   private final RigidBodyTransform mapToDepthTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];

   // Page-locked host memory for per-image transform pairs.  One slot per image index so that
   // kernels launched asynchronously on the same stream can read their own transform while the
   // CPU prepares the next one (same reason as VoxelMapExtractor's transformPointers list).
   private final List<FloatPointer> depthToMapTransformPointers = new ArrayList<>();
   private final List<FloatPointer> mapToDepthTransformPointers = new ArrayList<>();

   // Page-locked host buffer pre-filled with FREE_SPACE_VALUE (+1.0f), used to init the GPU TSDF
   // buffer each call via cudaMemcpyAsync (cudaMemset works on bytes, +1.0f ≠ 0x00).
   private final FloatPointer freespaceInitBuffer;

   private int error;

   public TSDFVoxelMapExtractor(int mapSize, float voxelSize, float truncationDistance)
   {
      this.mapSize = mapSize;
      this.voxelSize = voxelSize;
      this.truncationDistance = truncationDistance;
      this.voxelCount = mapSize * mapSize * mapSize;

      URL tsdfURL = CUDATools.class.getResource("VoxelTSDFExtraction.cu");
      URL utilsURL = CUDATools.getUtilsFile();
      URL perceptionUtilsURL = CUDATools.getPerceptionUtilsFile();
      URL mathUtilsURL = CUDATools.class.getResource("MathUtils.cuh");

      try
      {
         program = new CUDAProgram(tsdfURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         fuseDepthImageKernel = program.loadKernel("fuseDepthImageToTSDF");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      stream = CUDAStreamManager.getStream();

      // Allocate page-locked host buffer and fill with FREE_SPACE_VALUE once
      freespaceInitBuffer = new FloatPointer();
      error = cudaMallocHost(freespaceInitBuffer, (long) Float.BYTES * voxelCount);
      CUDATools.checkCUDAError(error);
      for (int i = 0; i < voxelCount; i++)
         freespaceInitBuffer.put(i, FREE_SPACE_VALUE);
   }

   /**
    * Fuses the given depth images into a TSDF voxel map centered on {@code origin} (robot pelvis pose).
    *
    * <p>Each camera's contribution uses the pelvis-anchored signed-distance convention:
    * {@code tsdf = clamp((||surface_map|| - ||voxel_map||) / truncationDistance, -1, 1)}
    * where both distances are measured from the map-frame origin (the pelvis).  Multi-camera fusion
    * takes the minimum (most-occluded wins), matching {@code depth_voxel_tsdf.py} exactly.</p>
    *
    * <p>The returned {@link TSDFVoxelMap} owns its GPU memory; the caller must call
    * {@link TSDFVoxelMap#close()} when done.  Depth images are released inside this method.</p>
    *
    * @param origin      pelvis pose in world frame (world_T_map), used as the map center and
    *                    the reference point for pelvis-anchored TSDF distances
    * @param depthImages 16-bit depth images (depth value 0 = no measurement)
    * @return TSDF grid on GPU (and lazily on CPU via {@link TSDFVoxelMap#getCpuData()})
    */
   public TSDFVoxelMap getTSDFVoxelMap(RigidBodyTransformReadOnly origin, RawImage... depthImages)
   {
      FloatPointer tsdfPointer = new FloatPointer();
      CUDATools.mallocAsync(tsdfPointer, voxelCount, stream);
      // Initialize all voxels to FREE_SPACE_VALUE (+1.0f); cudaMemset cannot write floats directly
      error = cudaMemcpyAsync(tsdfPointer, freespaceInitBuffer, (long) Float.BYTES * voxelCount, cudaMemcpyHostToDevice, stream);
      CUDATools.checkCUDAError(error);

      BytePointer validPointer = new BytePointer();
      CUDATools.mallocAsync(validPointer, voxelCount, stream);
      error = cudaMemsetAsync(validPointer, 0, voxelCount, stream);
      CUDATools.checkCUDAError(error);

      // worldToMapTransform = map_T_world (maps world points into the pelvis/map frame)
      worldToMapTransform.set(origin);
      worldToMapTransform.invert();

      int gridSize = (voxelCount + BLOCK_SIZE - 1) / BLOCK_SIZE;
      int imageIndex = 0;

      for (RawImage depthImage : depthImages)
      {
         if (depthImage == null || depthImage.get() == null)
            continue;

         // depthToMapTransform = map_T_world * world_T_depth = map_T_depth
         depthToMapTransform.set(worldToMapTransform);
         depthToMapTransform.multiply(depthImage.getTransformToWorld());

         // mapToDepthTransform = depth_T_map (used to project voxel centers into the camera)
         mapToDepthTransform.set(depthToMapTransform);
         mapToDepthTransform.invert();

         FloatPointer dtmPtr = getOrAllocateTransformPointer(depthToMapTransformPointers, imageIndex);
         depthToMapTransform.get(transformArray);
         dtmPtr.put(transformArray);

         FloatPointer mtdPtr = getOrAllocateTransformPointer(mapToDepthTransformPointers, imageIndex);
         mapToDepthTransform.get(transformArray);
         mtdPtr.put(transformArray);

         imageIndex++;

         try (dim3 blockDim = new dim3(BLOCK_SIZE, 1, 1);
              dim3 gridDim = new dim3(gridSize, 1, 1))
         {
            fuseDepthImageKernel.withPointer(depthImage.getCUDADataPointer())
                                .withLong(depthImage.getGpuImageMat().step())
                                .withInt(depthImage.getWidth())
                                .withInt(depthImage.getHeight())
                                .withFloat(depthImage.getFocalLengthX())
                                .withFloat(depthImage.getFocalLengthY())
                                .withFloat(depthImage.getPrincipalPointX())
                                .withFloat(depthImage.getPrincipalPointY())
                                .withFloat(depthImage.getDepthDiscretization())
                                .withPointer(mtdPtr)
                                .withPointer(dtmPtr)
                                .withInt(mapSize)
                                .withFloat(voxelSize)
                                .withFloat(truncationDistance)
                                .withPointer(tsdfPointer)
                                .withPointer(validPointer)
                                .run(stream, gridDim, blockDim, 0);
         }

         depthImage.release();
      }

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      return new TSDFVoxelMap(null, tsdfPointer, null, validPointer, mapSize, voxelSize, origin);
   }

   private FloatPointer getOrAllocateTransformPointer(List<FloatPointer> list, int index)
   {
      while (list.size() <= index)
      {
         FloatPointer ptr = new FloatPointer();
         error = cudaMallocHost(ptr, 16L * Float.BYTES);
         CUDATools.checkCUDAError(error);
         list.add(ptr);
      }
      return list.get(index);
   }

   @Override
   public void close()
   {
      for (FloatPointer ptr : depthToMapTransformPointers)
         CUDATools.checkCUDAError(cudaFreeHost(ptr));
      for (FloatPointer ptr : mapToDepthTransformPointers)
         CUDATools.checkCUDAError(cudaFreeHost(ptr));

      CUDATools.checkCUDAError(cudaFreeHost(freespaceInitBuffer));
      freespaceInitBuffer.close();

      fuseDepthImageKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
