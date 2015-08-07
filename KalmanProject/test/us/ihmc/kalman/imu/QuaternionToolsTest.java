package us.ihmc.kalman.imu;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.robotics.geometry.QuaternionTools;
import Jama.Matrix;

public class QuaternionToolsTest
{
   @SuppressWarnings("unused")
   private static final java.text.DecimalFormat decimalFormat = new java.text.DecimalFormat();

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

   @EstimatedDuration
   @Test(timeout = 300000)
   public void testEuler2quat()
   {
//    double[][] eulerArray = new double[][]{{-1.782}, {-2.762}, {-2.629}};
//    Matrix euler = new Matrix(eulerArray);
//    Matrix quaternion = QuaternionTools.euler2quat(euler);
//
////    double[] expectedAnswer = new double[]{-0.709, -0.634, -0.015, -0.308};
//    double[] expectedAnswer = new double[]{-0.709, -0.634, -0.308, -0.015}; // From euclideanspace
//
//    for(int i=0; i<4; i++)
//    {
//       assertEquals(quaternion.get(i,0), expectedAnswer[i], 1e-3);
//    }
   }

   @EstimatedDuration
   @Test(timeout = 300000)
   public void testQuat2euler()
   {
//    Matrix quat = null;
//    Matrix expectedReturn = null;
//    Matrix actualReturn = QuaternionTools.quat2euler(quat);
//    assertEquals("return value", expectedReturn, actualReturn);
//    /**@todo fill in the test code*/
   }

   @Ignore
   @QuarantinedTest("Pretty old and uses Roll Pitch Yaw which is always flaky.")
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testBackAndForthOne()
   {
      Random random = new Random(1776L);
      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
//       double minAngle = -Math.PI;
//       double maxAngle = Math.PI;

         // If we limit pitch to +-PI/2 then things seem to work...
         double roll = nextDouble(random, -Math.PI, Math.PI);

//       double pitch = nextDouble(random, -Math.PI/2.0, Math.PI/2.0);
         double pitch = nextDouble(random, Math.PI / 2.0, Math.PI);
         double yaw = nextDouble(random, -Math.PI, Math.PI);

//       roll = 0.0;
//       pitch = -0.5 * Math.PI;
//       yaw = 0.5;

         double[][] eulerArray = new double[][]
         {
            {roll}, {pitch}, {yaw}
         };
         Matrix euler = new Matrix(eulerArray);

//       Matrix quaternion = QuaternionTools.euler2quat(euler);
//       Matrix eulerBack = QuaternionTools.quat2euler(quaternion);

         Matrix quaternion = new Matrix(4, 1);
         Matrix eulerBack = new Matrix(3, 1);

         QuaternionTools.rollPitchYawToQuaternions(euler, quaternion);
         QuaternionTools.quaternionsToRollPitchYaw(quaternion, eulerBack);

         boolean passed = true;
         for (int j = 0; j < 3; j++)
         {
            double difference = QuaternionTools.computeAngleDifferenceMinusPiToPi(euler.get(j, 0), eulerBack.get(j, 0));

            passed = passed && (Math.abs(difference) < 1e-7);
         }

         if (!passed)
         {
            System.out.println("testBackAndForthOne() Faile!!!\neuler:");
            euler.print(10, 10);

            System.out.println("quaternion:");
            quaternion.print(10, 10);

            System.out.println("eulerBack:");
            eulerBack.print(10, 10);

            throw new RuntimeException("testBackAndForthOne() failed!");
         }
      }
   }

   @Ignore
   @QuarantinedTest("Pretty old and uses Roll Pitch Yaw which is always flaky.")
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testBackAndForthTwo()
   {
      Random random = new Random(1776L);
      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         double q0 = nextDouble(random, 0.0, 1.0);
         double q1 = nextDouble(random, -1.0, 1.0);
         double q2 = nextDouble(random, -1.0, 1.0);
         double q3 = nextDouble(random, -1.0, 1.0);

         double[][] quaternionArray = new double[][]
         {
            {q0}, {q1}, {q2}, {q3}
         };
         Matrix quaternion = new Matrix(quaternionArray);
         FullIMUKalmanFilter.normalize(quaternion);

         Matrix euler = new Matrix(3, 1);
         Matrix quaternionBack = new Matrix(4, 1);

         QuaternionTools.quaternionsToRollPitchYaw(quaternion, euler);
         QuaternionTools.rollPitchYawToQuaternions(euler, quaternionBack);

         boolean passed = true;
         for (int j = 0; j < 4; j++)
         {
            // Quaternion is equivalent to negative quaternion (its reflexion)
            if (quaternion.get(0, 0) * quaternionBack.get(0, 0) < 0.0)
            {
               quaternionBack = quaternionBack.times(-1.0);
            }

            double difference = quaternionBack.get(j, 0) - quaternion.get(j, 0);

            passed = passed && (Math.abs(difference) < 1e-7);
         }

         if (!passed)
         {
            System.out.println("testBackAndForthTwo() Failed!!!\nquaternion:");
            quaternion.print(10, 10);

            System.out.println("euler:");
            euler.print(10, 10);

            System.out.println("quaternionBack:");
            quaternionBack.print(10, 10);

            throw new RuntimeException("testBackAndForthTwo() failed!");
         }


         for (int j = 0; j < 4; j++)
         {
            assertEquals(quaternion.get(j, 0), quaternionBack.get(j, 0), 1e-7);
         }
      }
   }


   private double nextDouble(Random random, double min, double max)
   {
      return min + (max - min) * random.nextDouble();
   }

   @SuppressWarnings("unused")
   private boolean epsilonEquals(double a, double b, double epsilon)
   {
      return (Math.abs(a - b) <= epsilon);
   }
}
