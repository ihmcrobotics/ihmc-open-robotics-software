package us.ihmc.graphics3DAdapter;

import java.util.concurrent.LinkedBlockingQueue;

import javax.media.j3d.Transform3D;

import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.LidarScanParameters;

@SuppressWarnings("serial")
public class GPULidarScanBuffer extends LinkedBlockingQueue<LidarScan> implements GPULidarListener
{
   private final LidarScanParameters parameters;
   
   public GPULidarScanBuffer(LidarScanParameters parameters)
   {
      this.parameters = parameters;
   }
   
   @Override
   public void scan(float[] scan, Transform3D currentTransform, double time)
   {
      add(new LidarScan(parameters, new Transform3D(currentTransform),
               new Transform3D(currentTransform), scan));
   }
}
