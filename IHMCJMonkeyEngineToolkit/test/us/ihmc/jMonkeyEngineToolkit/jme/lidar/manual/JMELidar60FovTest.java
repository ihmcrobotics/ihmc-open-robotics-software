package us.ihmc.jMonkeyEngineToolkit.jme.lidar.manual;

import us.ihmc.jMonkeyEngineToolkit.jme.lidar.JMEGPULidarTestEnviroment;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.LidarTestListener;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.LidarTestParameters;
import us.ihmc.robotics.lidar.LidarScan;

public class JMELidar60FovTest extends LidarTestParameters implements LidarTestListener
{
   public static void main(String[] args)
   {
      JMELidar60FovTest test = new JMELidar60FovTest();
      new JMEGPULidarTestEnviroment().testManually(test, test);
   }

   public JMELidar60FovTest()
   {
      setShowGpuPoints(false);
      setPlaceLidar(false);
      setShowScanRays(false);
      setRotationSpeed(5.0);
      setScansPerSweep(720);
      setLidarTestRotationAmount((2 * Math.PI));
      setRotationLimitEnabled(true);
      setPrintDebug(false);

      setViewWidth(1000);
      setViewHeight(700);
      setShowSky(true);
      setWallDistance(-5.0);
      setPlaceJmeSphere(true);
      setPlaceIhmcSphere(false);
      setPlaceLidar(true);
      setPlaceWall(false);
      setRotateWall(false);
      setShowGpuPoints(true);
      setShowTracePoints(true);
      setShowScanRays(false);
      setGpuVsTraceTolerance(5.0);

      setLidarSweepStartAngle(-30 * Math.PI / 180);
      setLidarSweepEndAngle(30 * Math.PI / 180);
   }

   public void notify(LidarScan gpuScan, LidarScan traceScan)
   {
   }

   public void stop()
   {
   }
}
