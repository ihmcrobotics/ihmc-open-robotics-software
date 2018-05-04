package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

public interface LidarTestListener
{
   public void notify(LidarTestScan gpuScan, LidarTestScan traceScan);

   public void stop();
}
