package us.ihmc.perception.voxelMap;

import org.bytedeco.javacpp.FloatPointer;
import perception_msgs.VoxelMapMessage;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import static org.bytedeco.cuda.global.cudart.*;

public class VoxelMap
{
   private FloatPointer cpuData;
   private FloatPointer gpuData;

   private final int sizeX;
   private final int sizeY;
   private final int sizeZ;
   private final int voxelCount;

   private final float voxelSize;

   private final RigidBodyTransformReadOnly origin;

   public VoxelMap(FloatPointer cpuData, FloatPointer gpuData, int sizeX, int sizeY, int sizeZ, float voxelSize, RigidBodyTransformReadOnly origin)
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
      if (cpuData == null)
      {
         cpuData = new FloatPointer(voxelCount);
         cudaMemcpy(cpuData, gpuData, (long) gpuData.sizeof() * voxelCount, cudaMemcpyDeviceToHost);
      }

      return cpuData;
   }

   public FloatPointer getGpuData()
   {
      if (gpuData == null)
      {
         gpuData = new FloatPointer();
         cudaMalloc(gpuData, (long) cpuData.sizeof() * voxelCount);
         cudaMemcpy(gpuData, cpuData, (long) gpuData.sizeof() * voxelCount, cudaMemcpyHostToDevice);
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

      if (cpuData != null)
         cpuData.close();
   }

   public void toMessage(VoxelMapMessage messageToPack)
   {
      messageToPack.getVoxelMapData().clear();
      messageToPack.getVoxelMapData().ensureMinCapacity(voxelCount);
      getCpuData().get(messageToPack.getVoxelMapData().getBuffer().array());
      messageToPack.getVoxelMapData().getBuffer().position(voxelCount);

      messageToPack.setSizeX(sizeX);
      messageToPack.setSizeY(sizeY);
      messageToPack.setSizeZ(sizeZ);
      messageToPack.setVoxelSize(voxelSize);
      messageToPack.getOrigin().set(origin);
   }

   public static VoxelMap fromMessage(VoxelMapMessage message)
   {
      return new VoxelMap(new FloatPointer(message.getVoxelMapData().getBuffer()),
                          null,
                          message.getSizeX(),
                          message.getSizeY(),
                          message.getSizeZ(),
                          message.getVoxelSize(),
                          message.getOrigin().getPose());
   }
}
