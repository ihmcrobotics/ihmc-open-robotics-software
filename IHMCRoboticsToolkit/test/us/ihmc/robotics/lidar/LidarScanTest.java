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
}
