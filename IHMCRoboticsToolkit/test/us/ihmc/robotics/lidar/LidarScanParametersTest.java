package us.ihmc.robotics.lidar;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

public class LidarScanParametersTest
{
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_1()
   {
      LidarScanParameters lidarScanParameters = new LidarScanParameters();

      assertEquals(lidarScanParameters.getMaxRange(), 0, 1e-7);
      assertEquals(lidarScanParameters.getMinRange(), 0, 1e-7);
      assertEquals(lidarScanParameters.getMinRange(), 0, 1e-7);
      assertEquals(lidarScanParameters.pointsPerSweep, 0, 1e-7);
      assertEquals(lidarScanParameters.sweepYawMin, 0, 1e-7);
      assertEquals(lidarScanParameters.sweepYawMax, 0, 1e-7);
      assertEquals(lidarScanParameters.timeIncrement, 0, 1e-7);
      assertEquals(lidarScanParameters.scanTime, 0, 1e-7);
      assertEquals(lidarScanParameters.timestamp, 0, 1e-7);
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_2()
   {
      long timeStamp = System.currentTimeMillis();
      LidarScanParameters lidarScanParameters = new LidarScanParameters(new LidarScanParameters(), timeStamp);

      assertEquals(lidarScanParameters.getMaxRange(), 0, 1e-7);
      assertEquals(lidarScanParameters.getMinRange(), 0, 1e-7);
      assertEquals(lidarScanParameters.getMinRange(), 0, 1e-7);
      assertEquals(lidarScanParameters.pointsPerSweep, 0, 1e-7);
      assertEquals(lidarScanParameters.sweepYawMin, 0, 1e-7);
      assertEquals(lidarScanParameters.sweepYawMax, 0, 1e-7);
      assertEquals(lidarScanParameters.timeIncrement, 0, 1e-7);
      assertEquals(lidarScanParameters.scanTime, 0, 1e-7);
      assertEquals(lidarScanParameters.timestamp, timeStamp, 1e-7);
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_3()
   {
      Random rand = new Random();

      for (int i = 0; i < 1000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();

         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, fov, minRange, maxRange);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), 0 * 1E9, 1e-7);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
      }

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_4()
   {
      Random rand = new Random();

      for (int i = 0; i < 1000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();

         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, -fov / 2.0, fov / 2.0, minRange, maxRange);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), 0 * 1E9, 1e-7);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
      }

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_5()
   {
      Random rand = new Random();

      for (int i = 0; i < 1000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();

         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, -fov / 2.0, fov / 2.0, minRange, maxRange);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), 0 * 1E9, 1e-7);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
      }

   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_6()
   {
      Random rand = new Random();

      for (int i = 0; i < 1000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();
         double timeIncrement = rand.nextDouble();
         double scanTime = rand.nextDouble();
         double minYaw = -fov / 2.0;
         double maxYaw = fov / 2.0;

         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, minYaw, maxYaw, timeIncrement, minRange, maxRange, scanTime);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), timeIncrement, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), scanTime, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), scanTime * 1e9, 100);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
         assertEquals(lidarScanParameters.getAngleIncrement(), (maxYaw - minYaw)/pointsPerSweep, 1e-5);
      }

   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_7()
   {
      Random rand = new Random();

      for (int i = 0; i < 10000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();
         double timeIncrement = rand.nextDouble();
         double scanTime = rand.nextDouble();
         long timeStamp = System.currentTimeMillis();
         double minYaw = -fov / 2.0;
         double maxYaw = fov / 2.0;

         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, minYaw, maxYaw, timeIncrement, minRange, maxRange, scanTime);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), timeIncrement, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), scanTime, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), scanTime * 1e9, 100);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
         assertEquals(lidarScanParameters.getAngleIncrement(), (maxYaw - minYaw)/pointsPerSweep, 1e-3);
      }

   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_8()
   {
      Random rand = new Random();

      for (int i = 0; i < 1000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();
         double timeIncrement = rand.nextDouble();
         double scanTime = rand.nextDouble();
         long timeStamp = System.currentTimeMillis();
         double minYaw = -fov / 2.0;
         double maxYaw = fov / 2.0;
         
         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, minYaw, maxYaw, minRange, maxRange);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), 0 * 1e9, 100);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
         assertEquals(lidarScanParameters.getAngleIncrement(), (maxYaw - minYaw)/pointsPerSweep, 1e-3);
      }

   }
   
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_9()
   {
      Random rand = new Random();

      for (int i = 0; i < 1000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();
         double timeIncrement = rand.nextDouble();
         double scanTime = rand.nextDouble();
         long timeStamp = System.currentTimeMillis();
         double minYaw = -fov / 2.0;
         double maxYaw = fov / 2.0;
         
         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, (float)minYaw, (float)maxYaw, (float)minRange, (float)maxRange);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), 0, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), 0 * 1e9, 100);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
         assertEquals(lidarScanParameters.getAngleIncrement(), (maxYaw - minYaw)/pointsPerSweep, 1e-3);
      }

   }
   
//   public LidarScanParameters(int pointsPerSweep, float sweepYawMin, float sweepYawMax, float timeIncrement, float minRange, float maxRange, float scanTime)
//   {
//      this(pointsPerSweep, sweepYawMin, sweepYawMax, timeIncrement, minRange, maxRange, scanTime, 0l);
//   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void LidarScanParameters_10()
   {
      Random rand = new Random();

      for (int i = 0; i < 10000; i++)
      {
         int pointsPerSweep = rand.nextInt();
         int fov = (int) (rand.nextFloat() * Math.PI);
         double minRange = rand.nextDouble();
         double maxRange = rand.nextDouble();
         double timeIncrement = rand.nextDouble();
         double scanTime = rand.nextDouble();
         long timeStamp = System.currentTimeMillis();
         double minYaw = -fov / 2.0;
         double maxYaw = fov / 2.0;
         
         LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, (float)minYaw, (float)maxYaw, (float)timeIncrement, (float)minRange, (float)maxRange, (float)scanTime);

         assertEquals(lidarScanParameters.getMaxRange(), maxRange, 1e-7);
         assertEquals(lidarScanParameters.getMinRange(), minRange, 1e-7);
         assertEquals(lidarScanParameters.getPointsPerSweep(), pointsPerSweep, 1e-7);
         assertEquals(lidarScanParameters.getFieldOfView(), fov, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMin(), -fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getSweepYawMax(), fov / 2.0, 1e-7);
         assertEquals(lidarScanParameters.getTimeIncrement(), timeIncrement, 1e-7);
         assertEquals(lidarScanParameters.getScanTime(), scanTime, 1e-7);
         assertEquals(lidarScanParameters.getScanTimeNanos(), scanTime * 1e9, 100);
         assertEquals(lidarScanParameters.getTimestamp(), 0, 1e-7);
         assertEquals(lidarScanParameters.getAngleIncrement(), (maxYaw - minYaw)/pointsPerSweep, 1e-3);
      }

   }

}
