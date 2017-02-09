package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationTools.AxisAngleComparisonMode;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.random.RandomTools;

public class RotationToolsTest
{
   private Random random = new Random(100L);
   private static final double EPSILON = 1e-10;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisAngleEpsilonEqualsIgnoreCompleteRotations()
   {
      AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;

      for (int i = 0; i < 100; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);
         Vector3d randomAxis = RandomTools.generateRandomVector(random, 1.0);
         double randomAngleCompleteRotations = random.nextInt(2) * (2.0 * Math.PI);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxis, randomAngle);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxis, randomAngle + randomAngleCompleteRotations);

         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < 100; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);
         Vector3d randomAxis = RandomTools.generateRandomVector(random, 1.0);
         double randomAngleNotCompleteRotations = random.nextInt(2) * (2.0 * Math.PI) + 0.25 * AngleTools.generateRandomAngle(random);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxis, randomAngle);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxis, randomAngle + randomAngleNotCompleteRotations);

         assertTrue(axisAngleA + "\n should *NOT* equal:\n" + axisAngleB + "!", !RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisAngleEpsilonEqualsIgnoreFlippedAxes()
   {
      for (int i = 0; i < 100; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);
         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisA_flipped = new Vector3d(randomAxisA);
         randomAxisA_flipped.negate();

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, randomAngle);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisA_flipped, -randomAngle);

         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEqualsIgnoreFlippedAxes(axisAngleA, axisAngleB, EPSILON));
      }

      for (int i = 0; i < 100; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);

         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisA_flipped = new Vector3d(randomAxisA);
         randomAxisA_flipped.negate();

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, randomAngle);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisA_flipped, randomAngle);

         assertTrue(axisAngleA + "\n should *NOT* equal:\n" + axisAngleB + "!", !RotationTools.axisAngleEpsilonEqualsIgnoreFlippedAxes(axisAngleA, axisAngleB, EPSILON));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisAngleEpsilonEqualsAnglesAreZero()
   {
      int numberOfTests = 100;

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisB = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, 0.0);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisB, 0.0);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisAngleEpsilonEqualsAnglesDivisibleByTwoPi()
   {
      int numberOfTests = 100;
      AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;

      for (int i = 0; i < numberOfTests; i++)
      {
         double randomAngleA_MultipleOfTwoPi = random.nextInt(2) * (2.0 * Math.PI);
         double randomAngleB_MultipleOfTwoPi = random.nextInt(2) * (2.0 * Math.PI);

         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisB = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, randomAngleA_MultipleOfTwoPi);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisB, randomAngleB_MultipleOfTwoPi);

         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         double randomAngleMultipleOfTwoPi = random.nextInt(2) * (2.0 * Math.PI);
         double randomAngleEpsilonToHalfPiMinusEpsilon = MathTools.clipToMinMax(random.nextDouble(), 0.1, Math.PI / 2.0 - 0.1);
         double randomAngleNotZeroOrMultipleOfTwoPi = randomAngleMultipleOfTwoPi + randomAngleEpsilonToHalfPiMinusEpsilon;

         double remainder = randomAngleNotZeroOrMultipleOfTwoPi % (2.0 * Math.PI);
         assertTrue(randomAngleNotZeroOrMultipleOfTwoPi + " is divisible by 2*PI, but should not be!", Math.abs(remainder) != 0.0);

         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisB = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, randomAngleMultipleOfTwoPi);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisB, randomAngleNotZeroOrMultipleOfTwoPi);

         assertTrue(axisAngleA + "\n should *NOT* equal:\n" + axisAngleB + "!", !RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisAngleEpsilonEqualsMinusPI()
   {
      int numberOfTests = 100;

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3d randomAxis = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxis, -Math.PI);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxis, -Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisB = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, -Math.PI);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisB, -Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should *NOT* equal:\n" + axisAngleB + "!", !RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAxisAngleEpsilonEqualsPlusPI()
   {
      int numberOfTests = 100;

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3d randomAxis = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxis, Math.PI);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxis, Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3d randomAxisA = RandomTools.generateRandomVector(random, 1.0);
         Vector3d randomAxisB = RandomTools.generateRandomVector(random, 1.0);

         AxisAngle4d axisAngleA = new AxisAngle4d(randomAxisA, Math.PI);
         AxisAngle4d axisAngleB = new AxisAngle4d(randomAxisB, Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should *NOT* equal:\n" + axisAngleB + "!", !RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testConvertMatrixToAxisAngle()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 50;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      Matrix3d expectedRotationMatrix = new Matrix3d();
      AxisAngle4d actualAxisAngle = new AxisAngle4d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d quaternionToTest = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(quaternionToTest);

            RotationTools.convertMatrixToAxisAngle(expectedRotationMatrix, actualAxisAngle);
            // I'm using the fact that converting an axis-angle to a rotation matrix is trivial without edge case => safe.
            actualRotationMatrix.set(actualAxisAngle);

            assertTrue(expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testConvertMatrixToAxisAngle4f()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 50;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      Matrix3f expectedRotationMatrix = new Matrix3f();
      AxisAngle4f actualAxisAngle = new AxisAngle4f();
      Matrix3f actualRotationMatrix = new Matrix3f();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d quaternionToTest = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(quaternionToTest);

            RotationTools.convertMatrixToAxisAngle(expectedRotationMatrix, actualAxisAngle);
            // I'm using the fact that converting an axis-angle to a rotation matrix is trivial without edge case => safe.
            actualRotationMatrix.set(actualAxisAngle);

            // Can't use EPSILON here 
            assertTrue(expectedRotationMatrix.epsilonEquals(actualRotationMatrix, 1.0e-6f));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConvertMatrixToQuaternion()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 50;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      Matrix3d expectedRotationMatrix = new Matrix3d();
      Quat4d actualQuaternion = new Quat4d();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);

            RotationTools.convertMatrixToQuaternion(expectedRotationMatrix, actualQuaternion);

            // I think the computed quaternion should be exactly the same as the given one.
            // This is why I'm using only the Tuple4d.epsilonEquals() instead of the epsilonEqulas from RotationTools.
            assertTrue(expectedQuaternion.epsilonEquals(actualQuaternion, EPSILON));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConvertMatrixToQuaternionFloat()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 50;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      Matrix3d expectedRotationMatrix = new Matrix3d();
      Quat4f expectedQuaternionFloat = new Quat4f();
      Quat4f actualQuaternion = new Quat4f();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            expectedQuaternionFloat.set(expectedQuaternion);
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);

            RotationTools.convertMatrixToQuaternion(expectedRotationMatrix, actualQuaternion);

            // I think the computed quaternion should be exactly the same as the given one.
            // This is why I'm using only the Tuple4d.epsilonEquals() instead of the epsilonEqulas from RotationTools.
            assertTrue(expectedQuaternionFloat.epsilonEquals(actualQuaternion, (float) EPSILON));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.2)
   @Test(timeout = 30000)
   public void testConvertMatrixToYawPitchRoll()
   {
      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      double[] actualYawPitchRoll = new double[3];

      int numberOfIterations = 100;
      double startAngle = -2.0 * Math.PI;
      double endAngle = 2.0 * Math.PI;
      double dAngle = (endAngle - startAngle) / (numberOfIterations - 1);

      for (int yawIndex = 0; yawIndex < numberOfIterations; yawIndex++)
      {
         double yaw = yawIndex * dAngle + startAngle;
         for (int pitchIndex = 0; pitchIndex < numberOfIterations; pitchIndex++)
         {
            double pitch = pitchIndex * dAngle + startAngle;
            for (int rollIndex = 0; rollIndex < numberOfIterations; rollIndex++)
            {
               double roll = rollIndex * dAngle + startAngle;
               yawRotation.rotZ(yaw);
               pitchRotation.rotY(pitch);
               rollRotation.rotX(roll);
               expectedRotationMatrix.mul(yawRotation, pitchRotation);
               expectedRotationMatrix.mul(rollRotation);

               RotationTools.convertMatrixToYawPitchRoll(expectedRotationMatrix, actualYawPitchRoll);

               yawRotation.rotZ(actualYawPitchRoll[0]);
               pitchRotation.rotY(actualYawPitchRoll[1]);
               rollRotation.rotX(actualYawPitchRoll[2]);
               actualRotationMatrix.mul(yawRotation, pitchRotation);
               actualRotationMatrix.mul(rollRotation);

               assertTrue(expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON));
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testConvertQuaternionAsDoublesToYawPitchRoll()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      double[] actualYawPitchRoll = new double[3];

      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();

      double totalNumberOfTests = quaternionsToTest.length * quaternionsToTest[0].length;
      double successRate = 0.0;
      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);
            RotationTools.convertQuaternionToYawPitchRoll(expectedQuaternion.getX(), expectedQuaternion.getY(), expectedQuaternion.getZ(), expectedQuaternion.getW(), actualYawPitchRoll);

            yawRotation.rotZ(actualYawPitchRoll[0]);
            pitchRotation.rotY(actualYawPitchRoll[1]);
            rollRotation.rotX(actualYawPitchRoll[2]);
            actualRotationMatrix.mul(yawRotation, pitchRotation);
            actualRotationMatrix.mul(rollRotation);

            // Yaw-pitch-roll representation is sensitive to gimbal lock which screws up the whole thing basically.
            // The only way I've found is to ensure a high success rate. That pretty much sucks.
            boolean epsilonEquals = expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON);
            if (epsilonEquals)
               successRate += 1.0 / totalNumberOfTests;
         }
      }
      assertTrue(successRate > 0.9995);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.9)
   @Test(timeout = 30000)
   public void testConvertQuaternionToYawPitchRoll()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      double[] actualYawPitchRoll = new double[3];

      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();

      double totalNumberOfTests = quaternionsToTest.length * quaternionsToTest[0].length;
      double successRate = 0.0;
      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);
            RotationTools.convertQuaternionToYawPitchRoll(expectedQuaternion, actualYawPitchRoll);

            yawRotation.rotZ(actualYawPitchRoll[0]);
            pitchRotation.rotY(actualYawPitchRoll[1]);
            rollRotation.rotX(actualYawPitchRoll[2]);
            actualRotationMatrix.mul(yawRotation, pitchRotation);
            actualRotationMatrix.mul(rollRotation);

            // Yaw-pitch-roll representation is sensitive to gimbal lock which screws up the whole thing basically.
            // The only way I've found is to ensure a high success rate. That pretty much sucks.
            boolean epsilonEquals = expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON);
            if (epsilonEquals)
               successRate += 1.0 / totalNumberOfTests;
         }
      }
      assertTrue(successRate > 0.9995);
   }

   /**
    * This test prevents dramatic bugs, but not the precision is so shitty that it's not really meaningful.
    */
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testConvertQuaternionFloatToYawPitchRoll()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
      Quat4f expectedQuaternion = new Quat4f();
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      double[] actualYawPitchRoll = new double[3];

      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();

      double totalNumberOfTests = quaternionsToTest.length * quaternionsToTest[0].length;
      double successRate = 0.0;
      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            expectedQuaternion.set(quaternionsToTest[i][j]);
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);
            RotationTools.convertQuaternionToYawPitchRoll(expectedQuaternion, actualYawPitchRoll);

            yawRotation.rotZ(actualYawPitchRoll[0]);
            pitchRotation.rotY(actualYawPitchRoll[1]);
            rollRotation.rotX(actualYawPitchRoll[2]);
            actualRotationMatrix.mul(yawRotation, pitchRotation);
            actualRotationMatrix.mul(rollRotation);

            // Yaw-pitch-roll representation is sensitive to gimbal lock which screws up the whole thing basically.
            // The only way I've found is to ensure a high success rate. That pretty much sucks.
            boolean epsilonEquals = expectedRotationMatrix.epsilonEquals(actualRotationMatrix, 1.0e-6);
            if (epsilonEquals)
               successRate += 1.0 / totalNumberOfTests;
         }
      }
      System.out.println(successRate);
      assertTrue(successRate > 0.9991);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testConvertRotationVectorToAxisAngle()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      AxisAngle4d expectedAxisAngle = new AxisAngle4d();
      AxisAngle4d actualAxisAngle = new AxisAngle4d();
      Vector3d rotationVector = new Vector3d();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            expectedAxisAngle.set(expectedQuaternion);
            rotationVector.set(expectedAxisAngle.getX(), expectedAxisAngle.getY(), expectedAxisAngle.getZ());
            rotationVector.scale(expectedAxisAngle.getAngle());

            RotationTools.convertRotationVectorToAxisAngle(rotationVector, actualAxisAngle);

            assertTrue(expectedAxisAngle.epsilonEquals(actualAxisAngle, EPSILON));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testConvertRotationVectorToMatrix()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      AxisAngle4d expectedAxisAngle = new AxisAngle4d();
      Vector3d rotationVector = new Vector3d();

      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            expectedRotationMatrix.set(expectedQuaternion);
            expectedAxisAngle.set(expectedQuaternion);
            rotationVector.set(expectedAxisAngle.getX(), expectedAxisAngle.getY(), expectedAxisAngle.getZ());
            rotationVector.scale(expectedAxisAngle.getAngle());

            RotationTools.convertRotationVectorToMatrix(rotationVector, actualRotationMatrix);

            assertTrue(expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testConvertTransformToQuaternion()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 50;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      Quat4d actualQuaternion = new Quat4d();

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedTransform.setRotation(expectedQuaternion);

            RotationTools.convertTransformToQuaternion(expectedTransform, actualQuaternion);

            // I think the computed quaternion should be exactly the same as the given one.
            // This is why I'm using only the Tuple4d.epsilonEquals() instead of the epsilonEqulas from RotationTools.
            assertTrue(expectedQuaternion.epsilonEquals(actualQuaternion, EPSILON));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.2)
   @Test(timeout = 30000)
   public void testConvertTransformToYawPitchRoll()
   {
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();

      double[] actualYawPitchRoll = new double[3];

      int numberOfIterations = 100;
      double startAngle = -2.0 * Math.PI;
      double endAngle = 2.0 * Math.PI;
      double dAngle = (endAngle - startAngle) / (numberOfIterations - 1);

      for (int yawIndex = 0; yawIndex < numberOfIterations; yawIndex++)
      {
         double yaw = yawIndex * dAngle + startAngle;
         for (int pitchIndex = 0; pitchIndex < numberOfIterations; pitchIndex++)
         {
            double pitch = pitchIndex * dAngle + startAngle;
            for (int rollIndex = 0; rollIndex < numberOfIterations; rollIndex++)
            {
               double roll = rollIndex * dAngle + startAngle;
               expectedTransform.setRotationEulerAndZeroTranslation(roll, pitch, yaw);

               RotationTools.convertTransformToYawPitchRoll(expectedTransform, actualYawPitchRoll);

               actualTransform.setRotationEulerAndZeroTranslation(actualYawPitchRoll[2], actualYawPitchRoll[1], actualYawPitchRoll[0]);

               assertTrue(expectedTransform.epsilonEquals(actualTransform, EPSILON));
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.5)
   @Test(timeout = 30000)
   public void testConvertYawPitchRollToMatrix()
   {
      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();

      int numberOfIterations = 100;
      double startAngle = -2.0 * Math.PI;
      double endAngle = 2.0 * Math.PI;
      double dAngle = (endAngle - startAngle) / (numberOfIterations - 1);

      for (int yawIndex = 0; yawIndex < numberOfIterations; yawIndex++)
      {
         double yaw = yawIndex * dAngle + startAngle;
         for (int pitchIndex = 0; pitchIndex < numberOfIterations; pitchIndex++)
         {
            double pitch = pitchIndex * dAngle + startAngle;
            for (int rollIndex = 0; rollIndex < numberOfIterations; rollIndex++)
            {
               double roll = rollIndex * dAngle + startAngle;
               yawRotation.rotZ(yaw);
               pitchRotation.rotY(pitch);
               rollRotation.rotX(roll);
               expectedRotationMatrix.mul(yawRotation, pitchRotation);
               expectedRotationMatrix.mul(rollRotation);
               RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, actualRotationMatrix);
               assertTrue(expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON));
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testConvertYawPitchRollArrayToMatrix()
   {
      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Matrix3d actualRotationMatrix = new Matrix3d();
      double[] yawPitchRoll = new double[3];

      int numberOfIterations = 100;
      double startAngle = -2.0 * Math.PI;
      double endAngle = 2.0 * Math.PI;
      double dAngle = (endAngle - startAngle) / (numberOfIterations - 1);

      for (int yawIndex = 0; yawIndex < numberOfIterations; yawIndex++)
      {
         double yaw = yawIndex * dAngle + startAngle;
         for (int pitchIndex = 0; pitchIndex < numberOfIterations; pitchIndex++)
         {
            double pitch = pitchIndex * dAngle + startAngle;
            for (int rollIndex = 0; rollIndex < numberOfIterations; rollIndex++)
            {
               double roll = rollIndex * dAngle + startAngle;
               yawRotation.rotZ(yaw);
               pitchRotation.rotY(pitch);
               rollRotation.rotX(roll);
               expectedRotationMatrix.mul(yawRotation, pitchRotation);
               expectedRotationMatrix.mul(rollRotation);

               yawPitchRoll[0] = yaw;
               yawPitchRoll[1] = pitch;
               yawPitchRoll[2] = roll;
               RotationTools.convertYawPitchRollToMatrix(yawPitchRoll, actualRotationMatrix);
               assertTrue(expectedRotationMatrix.epsilonEquals(actualRotationMatrix, EPSILON));
            }
         }
      }
   }

   /**
    * Assumes that {@link RotationTools#convertMatrixToQuaternion(Matrix3d, Quat4d)} is tested.
    * Assumes that {@link RotationTools#quaternionEpsilonEquals(Quat4d, Quat4d, double)} is tested.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testConvertYawPitchRollToQuaternion()
   {
      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Quat4d expectedQuaternion = new Quat4d();
      Quat4d actualQuaternion = new Quat4d();

      int numberOfIterations = 100;
      double startAngle = -2.0 * Math.PI;
      double endAngle = 2.0 * Math.PI;
      double dAngle = (endAngle - startAngle) / (numberOfIterations - 1);

      for (int yawIndex = 0; yawIndex < numberOfIterations; yawIndex++)
      {
         double yaw = yawIndex * dAngle + startAngle;
         for (int pitchIndex = 0; pitchIndex < numberOfIterations; pitchIndex++)
         {
            double pitch = pitchIndex * dAngle + startAngle;
            for (int rollIndex = 0; rollIndex < numberOfIterations; rollIndex++)
            {
               double roll = rollIndex * dAngle + startAngle;
               yawRotation.rotZ(yaw);
               pitchRotation.rotY(pitch);
               rollRotation.rotX(roll);
               expectedRotationMatrix.mul(yawRotation, pitchRotation);
               expectedRotationMatrix.mul(rollRotation);

               RotationTools.convertMatrixToQuaternion(expectedRotationMatrix, expectedQuaternion);
               RotationTools.convertYawPitchRollToQuaternion(yaw, pitch, roll, actualQuaternion);
               assertTrue(RotationTools.quaternionEpsilonEquals(expectedQuaternion, actualQuaternion, EPSILON));
            }
         }
      }
   }

   /**
    * Assumes that {@link RotationTools#convertMatrixToQuaternion(Matrix3d, Quat4d)} is tested.
    * Assumes that {@link RotationTools#quaternionEpsilonEquals(Quat4d, Quat4d, double)} is tested.
    */
   @ContinuousIntegrationTest(estimatedDuration = 1.6)
   @Test(timeout = 30000)
   public void testConvertYawPitchRollArrayToQuaternion()
   {
      Matrix3d yawRotation = new Matrix3d();
      Matrix3d pitchRotation = new Matrix3d();
      Matrix3d rollRotation = new Matrix3d();
      Matrix3d expectedRotationMatrix = new Matrix3d();
      Quat4d expectedQuaternion = new Quat4d();
      Quat4d actualQuaternion = new Quat4d();
      double[] yawPitchRoll = new double[3];

      int numberOfIterations = 100;
      double startAngle = -2.0 * Math.PI;
      double endAngle = 2.0 * Math.PI;
      double dAngle = (endAngle - startAngle) / (numberOfIterations - 1);

      for (int yawIndex = 0; yawIndex < numberOfIterations; yawIndex++)
      {
         double yaw = yawIndex * dAngle + startAngle;
         for (int pitchIndex = 0; pitchIndex < numberOfIterations; pitchIndex++)
         {
            double pitch = pitchIndex * dAngle + startAngle;
            for (int rollIndex = 0; rollIndex < numberOfIterations; rollIndex++)
            {
               double roll = rollIndex * dAngle + startAngle;
               yawRotation.rotZ(yaw);
               pitchRotation.rotY(pitch);
               rollRotation.rotX(roll);
               expectedRotationMatrix.mul(yawRotation, pitchRotation);
               expectedRotationMatrix.mul(rollRotation);

               RotationTools.convertMatrixToQuaternion(expectedRotationMatrix, expectedQuaternion);

               yawPitchRoll[0] = yaw;
               yawPitchRoll[1] = pitch;
               yawPitchRoll[2] = roll;
               RotationTools.convertYawPitchRollToQuaternion(yawPitchRoll, actualQuaternion);
               assertTrue(RotationTools.quaternionEpsilonEquals(expectedQuaternion, actualQuaternion, EPSILON));
            }
         }
      }
   }

   /**
    * Assumes that {@link RotationTools#convertQuaternionToYawPitchRoll(Quat4d, double[])} is tested.
    * Assumes that {@link RotationTools#convertMatrixToYawPitchRoll(Matrix3d, double[])} is tested.
    */
   @ContinuousIntegrationTest(estimatedDuration = 1.4)
   @Test(timeout = 30000)
   public void testComputeYaw()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
      Matrix3d expectedRotationMatrix = new Matrix3d();

      double[] expectedYawPitchRoll = new double[3];

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);
            try
            {
               RotationTools.convertQuaternionToYawPitchRoll(expectedQuaternion, expectedYawPitchRoll);
               assertEquals(expectedYawPitchRoll[0], RotationTools.computeYaw(expectedQuaternion), EPSILON);
            }
            catch (RuntimeException e)
            {
               // Could not compute the pitch rotation... :(
            }
            try
            {
               RotationTools.convertMatrixToYawPitchRoll(expectedRotationMatrix, expectedYawPitchRoll);
               assertEquals(expectedYawPitchRoll[0], RotationTools.computeYaw(expectedRotationMatrix), EPSILON);
            }
            catch (RuntimeException e)
            {
               // Could not compute the pitch rotation... :(
            }
         }
      }
   }

   /**
    * Assumes that {@link RotationTools#convertQuaternionToYawPitchRoll(Quat4d, double[])} is tested.
    */
   @ContinuousIntegrationTest(estimatedDuration = 1.2)
   @Test(timeout = 30000)
   public void testComputePitch()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
      Matrix3d expectedRotationMatrix = new Matrix3d();

      double[] expectedYawPitchRoll = new double[3];

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);
            RotationTools.convertQuaternionToYawPitchRoll(expectedQuaternion, expectedYawPitchRoll);
            try
            {
               assertEquals(expectedYawPitchRoll[1], RotationTools.computePitch(expectedQuaternion), EPSILON);
            }
            catch (RuntimeException e)
            {
               // Could not compute the pitch rotation... :(
            }
            try
            {
               assertEquals(expectedYawPitchRoll[1], RotationTools.computePitch(expectedRotationMatrix), EPSILON);
            }
            catch (RuntimeException e)
            {
               // Could not compute the pitch rotation... :(
            }
         }
      }
   }

   /**
    * Assumes that {@link RotationTools#convertQuaternionToYawPitchRoll(Quat4d, double[])} is tested.
    * Assumes that {@link RotationTools#convertMatrixToYawPitchRoll(Matrix3d, double[])} is tested.
    */
   @ContinuousIntegrationTest(estimatedDuration = 1.5)
   @Test(timeout = 30000)
   public void testComputeRoll()
   {
      int numberOfRays = 10000;
      int numberOfRotationsAroundRay = 100;
      Quat4d[][] quaternionsToTest = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
      Matrix3d expectedRotationMatrix = new Matrix3d();

      double[] expectedYawPitchRoll = new double[3];

      for (int i = 0; i < quaternionsToTest.length; i++)
      {
         for (int j = 0; j < quaternionsToTest[i].length; j++)
         {
            Quat4d expectedQuaternion = quaternionsToTest[i][j];
            // The conversion from quaternion to matrix is trivial => safe.
            expectedRotationMatrix.set(expectedQuaternion);
            try
            {
               RotationTools.convertQuaternionToYawPitchRoll(expectedQuaternion, expectedYawPitchRoll);
               assertEquals(expectedYawPitchRoll[2], RotationTools.computeRoll(expectedQuaternion), EPSILON);
            }
            catch (RuntimeException e)
            {
               // Could not compute the pitch rotation... :(
            }
            try
            {
               RotationTools.convertMatrixToYawPitchRoll(expectedRotationMatrix, expectedYawPitchRoll);
               assertEquals(expectedYawPitchRoll[2], RotationTools.computeRoll(expectedRotationMatrix), EPSILON);
            }
            catch (RuntimeException e)
            {
               // Could not compute the pitch rotation... :(
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetYawFromQuaternion()
   {
      // Pure Yaw
      double yaw = 1.0;
      double pitch = 0.0;
      double roll = 0.0;
      Matrix3d rotationMatrixToPack = new Matrix3d();
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrixToPack);
      Quat4d quatToPack = new Quat4d();
      RotationTools.convertMatrixToQuaternion(rotationMatrixToPack, quatToPack);
      double yawSolution = RotationTools.computeYaw(quatToPack);
      assertEquals("Yaw is not correct", yaw, yawSolution, EPSILON);

      // Yaw, Pitch, and Roll
      yaw = 1.0;
      pitch = 0.5;
      roll = 0.7;

      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrixToPack);
      quatToPack.set(rotationMatrixToPack);
      yawSolution = RotationTools.computeYaw(quatToPack);
      assertEquals("Yaw is not correct", yaw, yawSolution, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetQuaternionFromYawAndZNormal()
   {
      double yaw = 1.0;
      double pitch = 0.4;
      double roll = 0.8;
      Matrix3d rotationMatrixToPack = new Matrix3d();
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrixToPack);
      Vector3d normal = new Vector3d();
      rotationMatrixToPack.getColumn(2, normal);
      Quat4d quatToPack = new Quat4d();
      RotationTools.convertMatrixToQuaternion(rotationMatrixToPack, quatToPack);

      Quat4d quatSolution = new Quat4d();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, normal, quatSolution);

      assertTrue(RotationTools.quaternionEpsilonEquals(quatToPack, quatSolution, EPSILON));

      yaw = -Math.PI - 0.01;
      pitch = Math.PI / 2.0 - 1e-3;
      roll = 0.8;

      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrixToPack);
      normal = new Vector3d();
      rotationMatrixToPack.getColumn(2, normal);
      quatToPack.set(rotationMatrixToPack);

      quatSolution = new Quat4d();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, normal, quatSolution);

      assertTrue(RotationTools.quaternionEpsilonEquals(quatToPack, quatSolution, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomGetQuaternionFromYawAndZNormal()
   {
      int numTests = 100;
      Random random = new Random(7362L);
      Vector3d normal = new Vector3d();
      Quat4d quatSolution = new Quat4d();

      for (int i = 0; i < numTests; i++)
      {
         Quat4d quatToPack = RandomTools.generateRandomQuaternion(random);
         Matrix3d rotationMatrixToPack = new Matrix3d();
         double yaw = RotationTools.computeYaw(quatToPack);
         rotationMatrixToPack.set(quatToPack);
         rotationMatrixToPack.getColumn(2, normal);

         RotationTools.computeQuaternionFromYawAndZNormal(yaw, normal, quatSolution);

         boolean quaternionsAreEqual = RotationTools.quaternionEpsilonEquals(quatToPack, quatSolution, EPSILON);
         assertTrue(quaternionsAreEqual);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testQuaternionStuff()
   {
      Quat4d q = new Quat4d();

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         double pi = Math.PI;
         double yaw = (random.nextDouble() - 0.5) * 2.0 * pi;
         double pitch = (random.nextDouble() - 0.5) * pi;
         double roll = (random.nextDouble() - 0.5) * 2.0 * pi;
         double[] yawPitchRoll = {yaw, pitch, roll};

         RotationTools.convertYawPitchRollToQuaternion(yaw, pitch, roll, q);
         double[] yawPitchRollBack = new double[3];
         RotationTools.convertQuaternionToYawPitchRoll(q, yawPitchRollBack);

         double epsilon = 1e-8;
         localAssertArrayEquals(yawPitchRoll, yawPitchRollBack, epsilon);
      }
   }

   private void testSetQuaternionBasedOnMatrix3withQuat(Quat4d qref)
   {
      //Quaternion to rotation matrix is numerically robust, use for the correct answer;
      Matrix3d mTest = new Matrix3d();
      qref.normalize();
      mTest.set(qref);

      Quat4d qtest = new Quat4d();
      //      switch("apache")
      {
         //         case "vecmath":
         //            qtest.set(mTest);
         //            break;
         //         case "apache":
         RotationTools.convertMatrixToQuaternion(mTest, qtest);
         //            break;
         //         case "jme":
         //            Quaternion jqtest = new Quaternion();
         //            jqtest.fromRotationMatrix(
         //                  (float)mTest.m00, (float)mTest.m01, (float)mTest.m02,
         //                  (float)mTest.m10, (float)mTest.m11, (float)mTest.m12,
         //                  (float)mTest.m20, (float)mTest.m21, (float)mTest.m22);
         //            qtest.set(jqtest.getX(),jqtest.getY(),jqtest.getZ(),jqtest.getW());
         //            break;
      }
      qtest.normalize();
      //      System.err.println(" ref" + qref+"\n got" + qtest);
      Quat4d qdiff = new Quat4d(qref);
      qdiff.inverse();
      qdiff.mul(qtest);
      qdiff.normalize();
      //      System.err.println(" net:" + qdiff);
      AxisAngle4d axDiff = new AxisAngle4d();
      axDiff.set(qdiff);
      Vector3d vdiff = new Vector3d(axDiff.getX() * axDiff.getAngle(), axDiff.getY() * axDiff.getAngle(), axDiff.getZ() * axDiff.getAngle());
      assertEquals(0, vdiff.length(), 1e-5);
   }

   //standard identity matrix

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQuaternionBasedOnMatrix_Case0()
   {
      Quat4d[] testCases = new Quat4d[] {new Quat4d(0, 0, 0, 1), new Quat4d(1, 0, 0, 0), new Quat4d(0, 1, 0, 0), new Quat4d(0, 0, 1, 0), new Quat4d(0.559509264745704, 0.035077807528218076, -0.8227912676126732, -0.09345298295434751),
            new Quat4d(0.7133445472962442, -0.08371774492091577, -0.6603514244907018, -0.2192416297174736), new Quat4d(-0.7133445472962443, 0.08371774492091585, 0.6603514244907018, 0.2192416297174736),
            new Quat4d(-0.9917335356139193, 0.06733355650738714, 0.034281272593992815, 0.10370911655260683), new Quat4d(-0.9248702158006187, 0.08471834534956858, 0.36789564511740885, 0.04572395640984045)};

      for (int i = 0; i < testCases.length; i++)
      {
         //         System.err.println("testCase "+i);
         testSetQuaternionBasedOnMatrix3withQuat(testCases[i]);
         //         System.err.println("pass");
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSetQuaternionBasedOnMatrix3d()
   {
      Random random = new Random(1776L);

      Quat4d unitQuaternion = new Quat4d(0.0, 0.0, 0.0, 1.0);

      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         Quat4d randomQuaternion = RandomTools.generateRandomQuaternion(random);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(randomQuaternion);

         Quat4d quaternionToPack = new Quat4d();
         RotationTools.convertMatrixToQuaternion(rotationMatrix, quaternionToPack);

         quaternionToPack.mulInverse(randomQuaternion);

         if (quaternionToPack.getW() < 0.0)
            quaternionToPack.negate();

         boolean quaternionsAreEpsilonEquals = unitQuaternion.epsilonEquals(quaternionToPack, 1e-7);
         assertTrue(quaternionsAreEpsilonEquals);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testIntegrateToQuaternion() throws Exception
   {
      for (int i = 0; i < 100; i++)
      {
         Vector3d expectedAngularVelocity = RandomTools.generateRandomVector(random);
         Vector3d actualAngularVelocity = new Vector3d();

         Quat4d integrationResultPrevious = new Quat4d();
         Quat4d integrationResultCurrent = new Quat4d();
         Quat4d integrationResultNext = new Quat4d();
         Quat4d qDot = new Quat4d();

         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

         double dt = 1.0e-4;

         for (double t = dt; t <= 1.0; t += dt)
         {
            RotationTools.integrateAngularVelocity(expectedAngularVelocity, t - dt, integrationResultPrevious);
            RotationTools.integrateAngularVelocity(expectedAngularVelocity, t, integrationResultCurrent);
            RotationTools.integrateAngularVelocity(expectedAngularVelocity, t + dt, integrationResultNext);
            quaternionCalculus.computeQDotByFiniteDifferenceCentral(integrationResultPrevious, integrationResultNext, dt, qDot);
            quaternionCalculus.computeAngularVelocityInWorldFrame(integrationResultCurrent, qDot, actualAngularVelocity);

            assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, 1.0e-7));
         }
      }
   }

   private void localAssertArrayEquals(double[] expected, double[] actual, double epsilon)
   {
      for (int i = 0; i < expected.length; i++)
      {
         assertEquals(expected[i], actual[i], epsilon);
      }
   }

   /**
    * Test that has for only purpose to highlight a bug in Java3d.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 10000)
   public void testJava3dAxisAngleSetMatrixBug()
   {
      Matrix3d m = new Matrix3d(-0.9945629970516978, -0.10063678160888465, -0.02677093728187517, -0.10063683459913739, 0.8627481429886237, 0.49551777898633176, -0.026770738081164314, 0.4955177897483468, -0.8681851459369152);

      AxisAngle4d a = new AxisAngle4d();
      a.set(m);
      Matrix3d m2 = new Matrix3d();
      m2.set(a);

      assertTrue(m2.epsilonEquals(m, 1e-5));

   }

   /**
    * Test that has for only purpose to highlight a bug in Java3d
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 10000)
   public void testJava3dQuat4dSetMatrixBug()
   {
      Matrix3d m = new Matrix3d(-0.9945629970516978, -0.10063678160888465, -0.02677093728187517, -0.10063683459913739, 0.8627481429886237, 0.49551777898633176, -0.026770738081164314, 0.4955177897483468, -0.8681851459369152);

      Quat4d a = new Quat4d();
      a.set(m);
      Matrix3d m2 = new Matrix3d();
      m2.set(a);

      assertTrue(m2.epsilonEquals(m, 1e-5));

   }

   /**
    * Test that has for only purpose to highlight a bug in Java3d
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 10000)
   public void testJava3dQuat4dSetMatrixBug2()
   {
      Matrix3d m = new Matrix3d(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      Quat4d a = new Quat4d();
      a.set(m);
      Matrix3d m2 = new Matrix3d();
      m2.set(a);

      assertTrue(m2.epsilonEquals(m, 1e-5));

   }

   /**
    * Test that has for only purpose to highlight a bug in Java3d
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 10000)
   public void testJava3dAxisAngleSetMatrixBug2()
   {
      Matrix3d m = new Matrix3d(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      AxisAngle4d a = new AxisAngle4d();
      a.set(m);
      Matrix3d m2 = new Matrix3d();
      m2.set(a);

      System.out.println(m);
      System.out.println(m2);
      assertTrue(m2.epsilonEquals(m, 1e-5));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJava3dAxisAngle4dSetMatrixBugWorkAround()
   {
      Matrix3d m = new Matrix3d(-0.9945629970516978, -0.10063678160888465, -0.02677093728187517, -0.10063683459913739, 0.8627481429886237, 0.49551777898633176, -0.026770738081164314, 0.4955177897483468, -0.8681851459369152);
      AxisAngle4d a = new AxisAngle4d();
      RotationTools.convertMatrixToAxisAngle(m, a);
      Matrix3d m3 = new Matrix3d();
      m3.set(a);
      assertTrue(m3.epsilonEquals(m, 1e-5));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJava3dAxisAngle4fSetMatrixBugWorkAround()
   {
      Matrix3f m = new Matrix3f(-0.9945629970516978f, -0.10063678160888465f, -0.02677093728187517f, -0.10063683459913739f, 0.8627481429886237f, 0.49551777898633176f, -0.026770738081164314f, 0.4955177897483468f, -0.8681851459369152f);
      AxisAngle4f a = new AxisAngle4f();
      RotationTools.convertMatrixToAxisAngle(m, a);
      Matrix3f m3 = new Matrix3f();
      m3.set(a);
      assertTrue(m3.epsilonEquals(m, 1e-5f));
   }

   public static void assertAxisAngleEquivalent(String errorMsg, AxisAngle4d axisAngleExpected, AxisAngle4d axisAngleActual, AxisAngleComparisonMode mode, double epsilon)
   {
      boolean axisAnglesAreEqual = RotationTools.axisAngleEpsilonEquals(axisAngleExpected, axisAngleActual, epsilon, mode);
      assertTrue(errorMsg + "\n AxisAngles are not Equal!\n expected:\n<" + axisAngleExpected + ">\n but was:\n<" + axisAngleActual + ">", axisAnglesAreEqual);
   }
}
