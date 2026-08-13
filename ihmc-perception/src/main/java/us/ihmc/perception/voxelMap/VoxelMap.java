package us.ihmc.perception.voxelMap;

import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static org.bytedeco.cuda.global.cudart.*;

public class VoxelMap
{
   private FloatPointer cpuData;
   private final Lock cpuDataLock = new ReentrantLock();

   private FloatPointer gpuData;
   private final Lock gpuDataLock = new ReentrantLock();

   private final int sizeX;
   private final int sizeY;
   private final int sizeZ;
   private final int voxelCount;

   private final float voxelSize;

   private final Pose3D origin;

   public VoxelMap(FloatPointer cpuData, FloatPointer gpuData, int sizeX, int sizeY, int sizeZ, float voxelSize, Pose3D origin)
   {
      this.cpuData = cpuData;
      this.gpuData = gpuData;
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
      this.voxelSize = voxelSize;
      this.origin = origin;

      voxelCount = sizeX * sizeY * sizeZ;
   }

   public FloatPointer getCpuData()
   {
      cpuDataLock.lock();
      try
      {
         if (cpuData == null)
         {
            cpuData = new FloatPointer(voxelCount);
            cudaMemcpy(cpuData, gpuData, (long) gpuData.sizeof() * voxelCount, cudaMemcpyDeviceToHost);
         }
      }
      finally
      {
         cpuDataLock.unlock();
      }

      return cpuData;
   }

   public FloatPointer getGpuData()
   {
      gpuDataLock.lock();
      try
      {
         if (gpuData == null)
         {
            gpuData = new FloatPointer();
            cudaMalloc(gpuData, (long) cpuData.sizeof() * voxelCount);
            cudaMemcpy(gpuData, cpuData, (long) gpuData.sizeof() * voxelCount, cudaMemcpyDeviceToHost);
         }
      }
      finally
      {
         gpuDataLock.unlock();
      }

      return gpuData;
   }

   public int getSizeX()
   {
      return sizeX;
   }

   public int getSizeY()
   {
      return sizeY;
   }

   public int getSizeZ()
   {
      return sizeZ;
   }

   public int getVoxelCount()
   {
      return voxelCount;
   }

   public float getVoxelSize()
   {
      return voxelSize;
   }

   public Pose3DReadOnly getOrigin()
   {
      return origin;
   }
}
