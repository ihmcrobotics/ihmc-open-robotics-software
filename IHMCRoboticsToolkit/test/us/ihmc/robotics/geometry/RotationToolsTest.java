package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationTools.AxisAngleComparisonMode;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.random.RandomGeometry;

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
         Vector3D randomAxis = RandomGeometry.nextVector3D(random, 1.0);
         double randomAngleCompleteRotations = random.nextInt(2) * (2.0 * Math.PI);

         AxisAngle axisAngleA = new AxisAngle(randomAxis, randomAngle);
         AxisAngle axisAngleB = new AxisAngle(randomAxis, randomAngle + randomAngleCompleteRotations);

         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < 100; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);
         Vector3D randomAxis = RandomGeometry.nextVector3D(random, 1.0);
         double randomAngleNotCompleteRotations = random.nextInt(2) * (2.0 * Math.PI) + 0.25 * AngleTools.generateRandomAngle(random);

         AxisAngle axisAngleA = new AxisAngle(randomAxis, randomAngle);
         AxisAngle axisAngleB = new AxisAngle(randomAxis, randomAngle + randomAngleNotCompleteRotations);

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
         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisA_flipped = new Vector3D(randomAxisA);
         randomAxisA_flipped.negate();

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, randomAngle);
         AxisAngle axisAngleB = new AxisAngle(randomAxisA_flipped, -randomAngle);

         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEqualsIgnoreFlippedAxes(axisAngleA, axisAngleB, EPSILON));
      }

      for (int i = 0; i < 100; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);

         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisA_flipped = new Vector3D(randomAxisA);
         randomAxisA_flipped.negate();

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, randomAngle);
         AxisAngle axisAngleB = new AxisAngle(randomAxisA_flipped, randomAngle);

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
         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisB = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, 0.0);
         AxisAngle axisAngleB = new AxisAngle(randomAxisB, 0.0);

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

         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisB = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, randomAngleA_MultipleOfTwoPi);
         AxisAngle axisAngleB = new AxisAngle(randomAxisB, randomAngleB_MultipleOfTwoPi);

         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         double randomAngleMultipleOfTwoPi = random.nextInt(2) * (2.0 * Math.PI);
         double randomAngleEpsilonToHalfPiMinusEpsilon = MathTools.clamp(random.nextDouble(), 0.1, Math.PI / 2.0 - 0.1);
         double randomAngleNotZeroOrMultipleOfTwoPi = randomAngleMultipleOfTwoPi + randomAngleEpsilonToHalfPiMinusEpsilon;

         double remainder = randomAngleNotZeroOrMultipleOfTwoPi % (2.0 * Math.PI);
         assertTrue(randomAngleNotZeroOrMultipleOfTwoPi + " is divisible by 2*PI, but should not be!", Math.abs(remainder) != 0.0);

         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisB = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, randomAngleMultipleOfTwoPi);
         AxisAngle axisAngleB = new AxisAngle(randomAxisB, randomAngleNotZeroOrMultipleOfTwoPi);

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
         Vector3D randomAxis = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxis, -Math.PI);
         AxisAngle axisAngleB = new AxisAngle(randomAxis, -Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisB = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, -Math.PI);
         AxisAngle axisAngleB = new AxisAngle(randomAxisB, -Math.PI);

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
         Vector3D randomAxis = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxis, Math.PI);
         AxisAngle axisAngleB = new AxisAngle(randomAxis, Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should equal:\n" + axisAngleB + "!", RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         Vector3D randomAxisA = RandomGeometry.nextVector3D(random, 1.0);
         Vector3D randomAxisB = RandomGeometry.nextVector3D(random, 1.0);

         AxisAngle axisAngleA = new AxisAngle(randomAxisA, Math.PI);
         AxisAngle axisAngleB = new AxisAngle(randomAxisB, Math.PI);

         AxisAngleComparisonMode mode = AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
         assertTrue(axisAngleA + "\n should *NOT* equal:\n" + axisAngleB + "!", !RotationTools.axisAngleEpsilonEquals(axisAngleA, axisAngleB, EPSILON, mode));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetQuaternionFromYawAndZNormal()
   {
      double yaw = 1.0;
      double pitch = 0.4;
      double roll = 0.8;
      RotationMatrix rotationMatrixToPack = new RotationMatrix();
      rotationMatrixToPack.setYawPitchRoll(yaw, pitch, roll);
      Vector3D normal = new Vector3D();
      rotationMatrixToPack.getColumn(2, normal);
      Quaternion quatToPack = new Quaternion();
      quatToPack.set(rotationMatrixToPack);

      Quaternion quatSolution = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, normal, quatSolution);

      assertTrue(RotationTools.quaternionEpsilonEquals(quatToPack, quatSolution, EPSILON));

      yaw = -Math.PI - 0.01;
      pitch = Math.PI / 2.0 - 1e-3;
      roll = 0.8;

      rotationMatrixToPack.setYawPitchRoll(yaw, pitch, roll);
      normal = new Vector3D();
      rotationMatrixToPack.getColumn(2, normal);
      quatToPack.set(rotationMatrixToPack);

      quatSolution = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, normal, quatSolution);

      assertTrue(RotationTools.quaternionEpsilonEquals(quatToPack, quatSolution, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomGetQuaternionFromYawAndZNormal()
   {
      int numTests = 100;
      Random random = new Random(7362L);
      Vector3D normal = new Vector3D();
      Quaternion quatSolution = new Quaternion();

      for (int i = 0; i < numTests; i++)
      {
         Quaternion quatToPack = RandomGeometry.nextQuaternion(random);
         RotationMatrix rotationMatrixToPack = new RotationMatrix();
         double yaw = quatToPack.getYaw();
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
      Quaternion q = new Quaternion();

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         double pi = Math.PI;
         double yaw = (random.nextDouble() - 0.5) * 2.0 * pi;
         double pitch = (random.nextDouble() - 0.5) * 0.5 * pi;
         double roll = (random.nextDouble() - 0.5) * 2.0 * pi;
         double[] yawPitchRoll = {yaw, pitch, roll};

         q.setYawPitchRoll(yaw, pitch, roll);
         double[] yawPitchRollBack = new double[3];
         q.getYawPitchRoll(yawPitchRollBack);

         double epsilon = 1e-8;
         localAssertArrayEquals(yawPitchRoll, yawPitchRollBack, epsilon);
      }
   }

   private void testSetQuaternionBasedOnMatrix3withQuat(Quaternion qref)
   {
      //Quaternion to rotation matrix is numerically robust, use for the correct answer;
      RotationMatrix mTest = new RotationMatrix();
      qref.normalize();
      mTest.set(qref);

      Quaternion qtest = new Quaternion();
      //      switch("apache")
      {
         //         case "vecmath":
         //            qtest.set(mTest);
         //            break;
         //         case "apache":
         qtest.set(mTest);
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
      Quaternion qdiff = new Quaternion(qref);
      qdiff.inverse();
      qdiff.multiply(qtest);
      qdiff.normalize();
      //      System.err.println(" net:" + qdiff);
      AxisAngle axDiff = new AxisAngle();
      axDiff.set(qdiff);
      Vector3D vdiff = new Vector3D(axDiff.getX() * axDiff.getAngle(), axDiff.getY() * axDiff.getAngle(), axDiff.getZ() * axDiff.getAngle());
      assertEquals(0, vdiff.length(), 1e-5);
   }

   //standard identity matrix

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQuaternionBasedOnMatrix_Case0()
   {
      Quaternion[] testCases = new Quaternion[] {new Quaternion(0, 0, 0, 1), new Quaternion(1, 0, 0, 0), new Quaternion(0, 1, 0, 0), new Quaternion(0, 0, 1, 0), new Quaternion(0.559509264745704, 0.035077807528218076, -0.8227912676126732, -0.09345298295434751),
            new Quaternion(0.7133445472962442, -0.08371774492091577, -0.6603514244907018, -0.2192416297174736), new Quaternion(-0.7133445472962443, 0.08371774492091585, 0.6603514244907018, 0.2192416297174736),
            new Quaternion(-0.9917335356139193, 0.06733355650738714, 0.034281272593992815, 0.10370911655260683), new Quaternion(-0.9248702158006187, 0.08471834534956858, 0.36789564511740885, 0.04572395640984045)};

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

      Quaternion unitQuaternion = new Quaternion(0.0, 0.0, 0.0, 1.0);

      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {
         Quaternion randomQuaternion = RandomGeometry.nextQuaternion(random);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(randomQuaternion);

         Quaternion quaternionToPack = new Quaternion(rotationMatrix);

         quaternionToPack.multiplyConjugateOther(randomQuaternion);

         if (quaternionToPack.getS() < 0.0)
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
         Vector3D expectedAngularVelocity = RandomGeometry.nextVector3D(random);
         Vector3D actualAngularVelocity = new Vector3D();

         Quaternion integrationResultPrevious = new Quaternion();
         Quaternion integrationResultCurrent = new Quaternion();
         Quaternion integrationResultNext = new Quaternion();
         Vector4D qDot = new Vector4D();

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
      RotationMatrix m = new RotationMatrix(-0.9945629970516978, -0.10063678160888465, -0.02677093728187517, -0.10063683459913739, 0.8627481429886237, 0.49551777898633176, -0.026770738081164314, 0.4955177897483468, -0.8681851459369152);

      AxisAngle a = new AxisAngle();
      a.set(m);
      RotationMatrix m2 = new RotationMatrix();
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
      RotationMatrix m = new RotationMatrix(-0.9945629970516978, -0.10063678160888465, -0.02677093728187517, -0.10063683459913739, 0.8627481429886237, 0.49551777898633176, -0.026770738081164314, 0.4955177897483468, -0.8681851459369152);

      Quaternion a = new Quaternion();
      a.set(m);
      RotationMatrix m2 = new RotationMatrix();
      m2.set(a);

      assertTrue(m2.epsilonEquals(m, 1e-5));

   }

   public static void assertAxisAngleEquivalent(String errorMsg, AxisAngle axisAngleExpected, AxisAngle axisAngleActual, AxisAngleComparisonMode mode, double epsilon)
   {
      boolean axisAnglesAreEqual = RotationTools.axisAngleEpsilonEquals(axisAngleExpected, axisAngleActual, epsilon, mode);
      assertTrue(errorMsg + "\n AxisAngles are not Equal!\n expected:\n<" + axisAngleExpected + ">\n but was:\n<" + axisAngleActual + ">", axisAnglesAreEqual);
   }
}
