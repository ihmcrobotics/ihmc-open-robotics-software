package us.ihmc.graphics3DAdapter.jme.lidar;

import us.ihmc.utilities.lidar.polarLidar.LidarScan;

public interface LidarTestListener
{
   public void notify(LidarScan gpuScan, LidarScan traceScan);

   public void stop();
}
