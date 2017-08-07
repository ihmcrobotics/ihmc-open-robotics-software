package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import us.ihmc.robotics.lidar.LidarScan;

public interface LidarTestListener
{
   public void notify(LidarScan gpuScan, LidarScan traceScan);

   public void stop();
}
