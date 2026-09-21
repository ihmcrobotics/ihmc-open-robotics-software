package us.ihmc.perception.voxelMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Robot-centric TSDF voxel grid, produced by {@link TSDFVoxelMapExtractor}.
 *
 * Stores the TSDF float values (in [-1, 1]) and a per-voxel validity byte (0 = unobserved,
 * 1 = observed by at least one camera), both on GPU.  CPU copies are fetched lazily on first access.
 *
 * Grid layout: N^3 elements, x-major z-fastest (flatIndex = (x * N + y) * N + z).
 */
public class TSDFVoxelMap
{
   private FloatPointer cpuData;
   private final FloatPointer gpuData;
   private byte[] cpuValidData;
   private final BytePointer gpuValidData;

   private final int size;
   private final int voxelCount;
   private final float voxelSize;
   private final RigidBodyTransformReadOnly origin;

   TSDFVoxelMap(FloatPointer cpuData,
                FloatPointer gpuData,
                byte[] cpuValidData,
                BytePointer gpuValidData,
                int size,
                float voxelSize,
                RigidBodyTransformReadOnly origin)
   {
      this.cpuData = cpuData;
      this.gpuData = gpuData;
      this.cpuValidData = cpuValidData;
      this.gpuValidData = gpuValidData;
      this.size = size;
      this.voxelCount = size * size * size;
      this.voxelSize = voxelSize;
      this.origin = origin;
   }

   /** TSDF values in [-1, 1], N^3 floats.  Fetched from GPU on first call; cached thereafter. */
   public float[] getCpuData()
   {
      if (cpuData == null)
      {
         cpuData = new FloatPointer(voxelCount);
         cudaMemcpy(cpuData, gpuData, (long) Float.BYTES * voxelCount, cudaMemcpyDeviceToHost);
      }
      float[] array = new float[voxelCount];
      cpuData.get(array);
      return array;
   }

   /** Per-voxel validity bytes (0 = unobserved, 1 = observed).  Fetched from GPU on first call. */
   public byte[] getValidCpuData()
   {
      if (cpuValidData == null)
      {
         try (BytePointer hostPtr = new BytePointer(voxelCount))
         {
            cudaMemcpy(hostPtr, gpuValidData, voxelCount, cudaMemcpyDeviceToHost);
            cpuValidData = new byte[voxelCount];
            hostPtr.get(cpuValidData);
         }
      }
      return cpuValidData;
   }

   public FloatPointer getGpuData()
   {
      return gpuData;
   }

   public BytePointer getGpuValidData()
   {
      return gpuValidData;
   }

   public int getSize()
   {
      return size;
   }

   public int getVoxelCount()
   {
      return voxelCount;
   }

   public float getVoxelSize()
   {
      return voxelSize;
   }

   public RigidBodyTransformReadOnly getOrigin()
   {
      return origin;
   }

   public void close()
   {
      if (gpuData != null)
      {
         cudaFree(gpuData);
         gpuData.close();
      }
      if (gpuValidData != null)
      {
         cudaFree(gpuValidData);
         gpuValidData.close();
      }
      if (cpuData != null)
         cpuData.close();
   }
}
