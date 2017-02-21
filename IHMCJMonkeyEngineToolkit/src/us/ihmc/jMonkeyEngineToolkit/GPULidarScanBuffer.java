package us.ihmc.jMonkeyEngineToolkit;

import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;

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
