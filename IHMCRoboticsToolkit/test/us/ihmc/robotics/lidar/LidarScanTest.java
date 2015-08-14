package us.ihmc.robotics.lidar;

import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.Random;

import org.apache.commons.lang3.ArrayUtils;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransformTest;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.test.JUnitTools;
import us.ihmc.tools.thread.RunnableThatThrows;

public class LidarScanTest
{
	
	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testFlipNew()
   {
      float[] ranges = new float[720];
      
      Random random = new Random();
      for (int i = 0; i < 720; i++)
      {
         ranges[i] = random.nextFloat();
      }
      
      float[] flippedRanges = Arrays.copyOf(ranges, 720);
      
      ArrayUtils.reverse(flippedRanges);
      
      LidarScan lidarScan = new LidarScan(new LidarScanParameters(), new RigidBodyTransform(), new RigidBodyTransform(), ranges, 2);
      
      LidarScan flippedScan = new LidarScan(new LidarScanParameters(), new RigidBodyTransform(), new RigidBodyTransform(), flippedRanges, 2);
      
      assertLidarScanEquals(flippedScan, lidarScan.flipNew(), 1e-7, 1e-7f);
      
      assertLidarScanEquals(lidarScan, lidarScan.flipNew().flipNew(), 1e-7, 1e-7f);
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testAssertLidarScanRangesEqual()
   {
      Random random = new Random();

      float[] ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);
      float[] ranges2 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);

      final LidarScan lidarScan1 = new LidarScan(new LidarScanParameters(), new RigidBodyTransform(), new RigidBodyTransform(), ranges1, random.nextInt());
      final LidarScan lidarScan2 = new LidarScan(new LidarScanParameters(), new RigidBodyTransform(), new RigidBodyTransform(), ranges2, random.nextInt());

      JUnitTools.assertExceptionThrown(AssertionError.class, new RunnableThatThrows()
      {
         public void run() throws Throwable
         {
            assertLidarScanRangesEqual(lidarScan1, lidarScan2, 1e-7);
         }
      });

      LidarScan lidarScan3 = new LidarScan(new LidarScanParameters(), new RigidBodyTransform(), new RigidBodyTransform(), ranges1, random.nextInt());

      assertLidarScanRangesEqual(lidarScan1, lidarScan3, 1e-7);

      float[] ranges1Shortened = ArrayUtils.subarray(ranges1, 0, 3000);

      LidarScan lidarScan4 = new LidarScan(new LidarScanParameters(), new RigidBodyTransform(), new RigidBodyTransform(), ranges1Shortened, random.nextInt());

      assertLidarScanRangesEqual(lidarScan1, lidarScan4, 1e-7);
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testAssertLidarScanTransformsEqual()
   {
      Random random = new Random();

      float[] ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);
      float[] ranges2 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);

      RigidBodyTransform randomTransform1 = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform randomTransform2 = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform randomTransform3 = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform randomTransform4 = RigidBodyTransform.generateRandomTransform(random);

      final LidarScan lidarScan1 = new LidarScan(new LidarScanParameters(), randomTransform1, randomTransform2, ranges1);
      final LidarScan lidarScan2 = new LidarScan(new LidarScanParameters(), randomTransform1, randomTransform2, ranges2);

      assertLidarScanTransformsEqual(lidarScan1, lidarScan2, 1e-7);

      final LidarScan lidarScan3 = new LidarScan(new LidarScanParameters(), randomTransform3, randomTransform4, ranges2);

      JUnitTools.assertExceptionThrown(AssertionError.class, new RunnableThatThrows()
      {
         public void run() throws Throwable
         {
            assertLidarScanTransformsEqual(lidarScan1, lidarScan3, 1e-7);
         }
      });
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testAssertLidarScanEquals()
   {
      Random random = new Random();

      float[] ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);

      float[] ranges1Shortened = ArrayUtils.subarray(ranges1, 0, 3000);

      RigidBodyTransform randomTransform1 = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform randomTransform2 = RigidBodyTransform.generateRandomTransform(random);

      final LidarScan lidarScan1 = new LidarScan(new LidarScanParameters(), randomTransform1, randomTransform2, ranges1);
      final LidarScan lidarScan2 = new LidarScan(new LidarScanParameters(), randomTransform1, randomTransform2, ranges1Shortened);

      assertLidarScanEquals(lidarScan1, lidarScan2, 1e-7, 1e-7f);
   }

   public static void assertLidarScanRangesEqual(LidarScan lidarScan1, LidarScan lidarScan2, double rangeTolerance)
   {
      for (int i = 0; i < lidarScan1.size() && i < lidarScan2.size(); i++)
      {
         double value = lidarScan1.getRange(i);
         double value1 = lidarScan2.getRange(i);
         if ((value > 0 && value < Double.POSITIVE_INFINITY) && (value1 > 0 && value1 < Double.POSITIVE_INFINITY))
         {
            Assert.assertEquals(LidarScanTest.class.getSimpleName() + ": lidar scans differ: ", lidarScan1.getRange(i), lidarScan2.getRange(i), rangeTolerance);
         }
      }
   }

   public static void assertLidarScanTransformsEqual(LidarScan lidarScan1, LidarScan lidarScan2, double transformTolerance)
   {
      RigidBodyTransformTest.assertTransformEquals(lidarScan1.getStartTransform(), lidarScan2.getStartTransform(), transformTolerance);
      RigidBodyTransformTest.assertTransformEquals(lidarScan1.getEndTransform(), lidarScan2.getEndTransform(), transformTolerance);
   }

   public static void assertLidarScanEquals(LidarScan lidarScan1, LidarScan lidarScan2, double transformTolerance, float rangeTolerance)
   {
      assertLidarScanTransformsEqual(lidarScan1, lidarScan2, transformTolerance);
      assertLidarScanRangesEqual(lidarScan1, lidarScan2, rangeTolerance);
   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      Random random = new Random();
      float[] ranges1;
      LidarScan lidarScan1 = null;
      for(int i = 0; i < 1000; i++)
      {
    	  ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);

          RigidBodyTransform randomTransform1 = RigidBodyTransform.generateRandomTransform(random);
          RigidBodyTransform randomTransform2 = RigidBodyTransform.generateRandomTransform(random);

          lidarScan1 = new LidarScan(new LidarScanParameters(), randomTransform1, randomTransform2, ranges1);
      }
      
      assertEquals(lidarScan1.size(), 720, 1e-7f);
   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor2()
   {
      Random random = new Random();
      float[] ranges1;
      LidarScan lidarScan1 = null;
      for(int i = 0; i < 1000; i++)
      {
    	  int id = random.nextInt();
    	  ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);
          lidarScan1 = new LidarScan(new LidarScanParameters(), ranges1, id);
          assertEquals(lidarScan1.size(), 720, 1e-7f);
          assertEquals(lidarScan1.getSensorId(), id);
      }
   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testGetRanges()
   {
      Random random = new Random();
      float[] ranges1;
      LidarScan lidarScan1 = null;
      for(int i = 0; i < 1000; i++)
      {
    	  int id = random.nextInt();
    	  ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);
          lidarScan1 = new LidarScan(new LidarScanParameters(), ranges1, id);
          assertEquals(lidarScan1.getRanges().length, ranges1.length, 1e-7f);
          
          for(int j = 0; j < ranges1.length; j++)
          {
              assertEquals(lidarScan1.getRanges()[j], ranges1[j], 1e-7f);
          }
      }
   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testGetCopy()
   {
      Random random = new Random();
      float[] ranges1;
      LidarScan lidarScan1 = null;
      for(int i = 0; i < 1000; i++)
      {
    	  int id = random.nextInt();
    	  ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);
          lidarScan1 = new LidarScan(new LidarScanParameters(), ranges1, id);
          LidarScan lidarScanCopy = lidarScan1.getCopy();
          
          for(int j = 0; j < ranges1.length; j++)
          {
              assertEquals(lidarScan1.getRanges()[j], lidarScanCopy.getRanges()[j], 1e-7f);
          }
      }
   }
   
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void testScanParameters()
   {
      Random random = new Random();
      float[] ranges1;
      LidarScan lidarScan1 = null;
      for(int i = 0; i < 1000; i++)
      {
    	  int id = random.nextInt();
    	  ranges1 = RandomTools.generateRandomFloatArray(random, 720, 0, 5000);
    	  
          int pointsPerSweep = random.nextInt();
          int fov = (int) (random.nextFloat() * Math.PI);
          double minRange = random.nextDouble();
          double maxRange = random.nextDouble();

          LidarScanParameters lidarScanParameters = new LidarScanParameters(pointsPerSweep, fov, minRange, maxRange);
    	  
          lidarScan1 = new LidarScan(lidarScanParameters, ranges1, id);
          LidarScan lidarScanCopy = lidarScan1.getCopy();
          
          for(int j = 0; j < ranges1.length; j++)
          {
              assertEquals(lidarScan1.getRanges()[j], lidarScanCopy.getRanges()[j], 1e-7f);
              assertEquals(lidarScan1.getScanParameters().getMaxRange(), maxRange, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getMinRange(), minRange, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getPointsPerSweep(), pointsPerSweep, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getFieldOfView(), fov, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getSweepYawMin(), -fov / 2.0, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getSweepYawMax(), fov / 2.0, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getTimeIncrement(), 0, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getScanTime(), 0, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getScanTimeNanos(), 0 * 1E9, 1e-7);
              assertEquals(lidarScan1.getScanParameters().getTimestamp(), 0, 1e-7);
          }
      }
   }
}
