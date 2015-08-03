package us.ihmc.graphics3DAdapter;

import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.utilities.lidar.LidarScan;
import us.ihmc.utilities.lidar.LidarScanParameters;
import us.ihmc.robotics.geometry.RigidBodyTransform;

@SuppressWarnings("serial")
public class GPULidarScanBuffer extends LinkedBlockingQueue<LidarScan> implements GPULidarListener
{
   private final LidarScanParameters parameters;
   
   public GPULidarScanBuffer(LidarScanParameters parameters)
   {
      this.parameters = parameters;
   }
   
   @Override
   public void scan(float[] scan, RigidBodyTransform currentTransform, double time)
   {
      add(new LidarScan(parameters, new RigidBodyTransform(currentTransform),
               new RigidBodyTransform(currentTransform), scan));
   }
}
