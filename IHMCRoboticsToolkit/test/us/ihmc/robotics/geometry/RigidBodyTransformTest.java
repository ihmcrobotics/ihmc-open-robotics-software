package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationFunctions.ComparisonMode;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.test.JUnitTools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;
import java.util.Random;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class RigidBodyTransformTest
{
   private static final boolean DEBUG = false;

   private final int nTests = 200;

   
   @EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetRotationAndZeroTranslationWithAxisAngle4d()
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      AxisAngle4d axisAngleToCheck = new AxisAngle4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      double epsilonAssert = 1e-8;
      ComparisonMode comparisonMode = ComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
      
      double[] thetaTest = generateArrayOfTestAnglesMinus2PiToPiButExcludePlusMinusPi();

      for (double theta : thetaTest)
      {
         axisAngle.x = Math.sin(theta);
         axisAngle.y = Math.cos(theta);
         axisAngle.z = 0;
         axisAngle.angle = theta;

         transform.setRotationAndZeroTranslation(axisAngle);
         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

      for (double theta : thetaTest)
      {
         axisAngle.x = 0;
         axisAngle.y = Math.cos(theta);
         axisAngle.z = Math.sin(theta);
         axisAngle.angle = theta;

         transform.setRotation(axisAngle);
         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

      for (double theta : thetaTest)
      {         
         axisAngle.x = Math.cos(theta);
         axisAngle.y = 0;
         axisAngle.z = Math.sin(theta);
         axisAngle.angle = theta;

         transform.setRotation(axisAngle);
         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithAxisAngle4dAndRandomVector3d()
   {
      Random random = new Random();
      Vector3d vector = new Vector3d();
      AxisAngle4d axisAngle = new AxisAngle4d();
      AxisAngle4d axisAngleToCheck = new AxisAngle4d();

      double epsilonAssert = 1e-8;
      ComparisonMode comparisonMode = ComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;

      double[] thetaTest = generateArrayOfTestAnglesMinus2PiToPiButExcludePlusMinusPi();

      for (double theta : thetaTest)
      {
         axisAngle.x = Math.sin(theta);
         axisAngle.y = Math.cos(theta);
         axisAngle.z = 0;
         axisAngle.angle = theta;

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, vector);
         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

      for (double theta : thetaTest)
      {
         axisAngle.x = 0;
         axisAngle.y = Math.cos(theta);
         axisAngle.z = Math.sin(theta);
         axisAngle.angle = theta;

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, vector);
         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

      for (double theta : thetaTest)
      {
         axisAngle.x = Math.cos(theta);
         axisAngle.y = 0;
         axisAngle.z = Math.sin(theta);
         axisAngle.angle = theta;

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, vector);
         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithAxisAngle4dAndZeroVector3d()
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      AxisAngle4d axisAngleToCheck = new AxisAngle4d();

      double epsilonAssert = 1e-8;
      ComparisonMode comparisonMode = ComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;

      double[] thetaTest = generateArrayOfTestAnglesMinus2PiToPiButExcludePlusMinusPi();
      
      for (double theta : thetaTest)
      {
         axisAngle.x = Math.sin(theta);
         axisAngle.y = Math.cos(theta);
         axisAngle.z = 0;
         axisAngle.angle = theta;

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, new Vector3d(0, 0, 0));

         transform.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

      for (double theta : thetaTest)
      {
         axisAngle.x = 0;
         axisAngle.y = Math.cos(theta);
         axisAngle.z = Math.sin(theta);
         axisAngle.angle = theta;

         RigidBodyTransform transform1 = new RigidBodyTransform(axisAngle, new Vector3d(0, 0, 0));

         transform1.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

      for (double theta : thetaTest)
      {
         axisAngle.x = Math.cos(theta);
         axisAngle.y = 0;
         axisAngle.z = Math.sin(theta);
         axisAngle.angle = theta;

         RigidBodyTransform transform2 = new RigidBodyTransform(axisAngle, new Vector3d(0, 0, 0));
         transform2.getRotation(axisAngleToCheck);

         RotationFunctionsTest.assertAxisAngleEquivalent("theta = " + theta, axisAngle, axisAngleToCheck, comparisonMode, epsilonAssert);
      }

   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithAxisAngle4fEpsilonToPiMinusEpsilon() 
   {
      Random random = new Random();
      AxisAngle4f axisAngle = new AxisAngle4f();
      AxisAngle4f axisAngleToCheck = new AxisAngle4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      double epsilon = 0.01;
      double epsilonAssert = 1e-4;

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = (float) Math.sin(theta);
         axisAngle.y = (float) Math.cos(theta);
         axisAngle.z = 0;
         axisAngle.angle = (float) theta;

         transform.setRotationAndZeroTranslation(axisAngle);
         transform.normalize();
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = 0;
         axisAngle.y = (float) Math.cos(theta);
         axisAngle.z = (float) Math.sin(theta);
         axisAngle.angle = (float) theta;

         transform.setRotationAndZeroTranslation(axisAngle);
         transform.normalize();
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = (float) Math.cos(theta);
         axisAngle.y = 0;
         axisAngle.z = (float) Math.sin(theta);
         axisAngle.angle = (float) theta;

         transform.setRotationAndZeroTranslation(axisAngle);
         transform.normalize();
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithAxisAngle4fAndRandomVector3fEpsilonToPiMinusEpsilon()  
   {
      Random random = new Random();
      AxisAngle4f axisAngle = new AxisAngle4f();
      AxisAngle4f axisAngleToCheck = new AxisAngle4f();
      Vector3f vector = new Vector3f();

      double epsilon = 0.01;
      double epsilonAssert = 1e-4;

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = (float) Math.sin(theta);
         axisAngle.y = (float) Math.cos(theta);
         axisAngle.z = 0;
         axisAngle.angle = (float) theta;

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, vector);
         transform.normalize();
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = 0;
         axisAngle.y = (float) Math.cos(theta);
         axisAngle.z = (float) Math.sin(theta);
         axisAngle.angle = (float) theta;

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, vector);
         transform.normalize();
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = (float) Math.cos(theta);
         axisAngle.y = 0;
         axisAngle.z = (float) Math.sin(theta);
         axisAngle.angle = (float) theta;

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, vector);
         transform.normalize();
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }

   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithAxisAngle4fAndZeroVector3fEpsilonToPiMinusEpsilon()
   {
      Random random = new Random();
      AxisAngle4f axisAngle = new AxisAngle4f();
      AxisAngle4f axisAngleToCheck = new AxisAngle4f();

      double epsilon = 0.01;
      double epsilonAssert = 1e-3;

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = (float) Math.sin(theta);
         axisAngle.y = (float) Math.cos(theta);
         axisAngle.z = 0;
         axisAngle.angle = (float) theta;

         RigidBodyTransform transform = new RigidBodyTransform(axisAngle, new Vector3f(0,
               0, 0));
         transform.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, 1e-3);
         assertEquals(axisAngle.y, axisAngleToCheck.y, 1e-3);
         assertEquals(axisAngle.z, axisAngleToCheck.z, 1e-3);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, 1e-3);
      }

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = 0;
         axisAngle.y = (float) Math.cos(theta);
         axisAngle.z = (float) Math.sin(theta);
         axisAngle.angle = (float) theta;

         RigidBodyTransform transform1 = new RigidBodyTransform(axisAngle, new Vector3f(0,
               0, 0));
         transform1.normalize();
         transform1.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, 1e-3);
         assertEquals(axisAngle.y, axisAngleToCheck.y, 1e-3);
         assertEquals(axisAngle.z, axisAngleToCheck.z, 1e-3);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, 1e-3);
      }

      for (int i = 0; i < nTests; i++)
      {
         double theta = getRandomAngleEpsilonToPiMinusEpsilon(random, epsilon);
         axisAngle.x = (float) Math.cos(theta);
         axisAngle.y = 0;
         axisAngle.z = (float) Math.sin(theta);
         axisAngle.angle = (float) theta;

         RigidBodyTransform transform2 = new RigidBodyTransform(axisAngle, new Vector3f(0,
               0, 0));
         transform2.normalize();
         transform2.getRotation(axisAngleToCheck);

         assertEquals(axisAngle.x, axisAngleToCheck.x, epsilonAssert);
         assertEquals(axisAngle.y, axisAngleToCheck.y, epsilonAssert);
         assertEquals(axisAngle.z, axisAngleToCheck.z, epsilonAssert);
         assertEquals(axisAngle.angle, axisAngleToCheck.angle, epsilonAssert);
      }

   }
	
   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void TestNormalize()
   {
      Random random = new Random();
      Matrix4d matrix = new Matrix4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      createRandomTransformationMatrix(matrix, random);
      messWithRotationMatrixOrthogonality(matrix, random);

      transform.set(matrix);

      transform.normalize();

      assertTrue(checkOrthogonality(transform));
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestGetTransformAsQuat4dAndVector3d()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Quat4d quatCheck = new Quat4d();
      Vector3d vec = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextDouble();
         quat1.y = random.nextDouble();
         quat1.z = random.nextDouble();
         quat1.w = random.nextDouble();
         quat1.normalize();

         transform.setRotationAndZeroTranslation(quat1);

         transform.get(quatCheck, vec);

         assertEquals(quat1.x, quatCheck.x, 1e-10);
         assertEquals(quat1.y, quatCheck.y, 1e-10);
         assertEquals(quat1.z, quatCheck.z, 1e-10);
         assertEquals(quat1.w, quatCheck.w, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuat4dAndVector3d()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Vector3d trans = new Vector3d();
      Quat4d quatCheck = new Quat4d();
      Vector3d vec = new Vector3d();

      for (int i = 0; i < nTests; i++)
      {
         randomizeVector(random, trans);
         quat1.x = random.nextDouble();
         quat1.y = random.nextDouble();
         quat1.z = random.nextDouble();
         quat1.w = random.nextDouble();
         quat1.normalize();

         RigidBodyTransform transform = new RigidBodyTransform(quat1, trans);

         transform.get(quatCheck, vec);

         assertEquals(quat1.x, quatCheck.x, 1e-10);
         assertEquals(quat1.y, quatCheck.y, 1e-10);
         assertEquals(quat1.z, quatCheck.z, 1e-10);
         assertEquals(quat1.w, quatCheck.w, 1e-10);
         assertEquals(vec.x, trans.x, 1e-10);
         assertEquals(vec.y, trans.y, 1e-10);
         assertEquals(vec.z, trans.z, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuat4fAndVector3f()
   {
      Random random = new Random();
      Quat4f quat1 = new Quat4f();
      Vector3f trans = new Vector3f();
      Quat4f quatCheck = new Quat4f();
      Vector3f vec = new Vector3f();

      for (int i = 0; i < nTests; i++)
      {
         randomizeVector(random, trans);
         quat1.x = random.nextFloat();
         quat1.y = random.nextFloat();
         quat1.z = random.nextFloat();
         quat1.w = random.nextFloat();
         quat1.normalize();

         RigidBodyTransform transform = new RigidBodyTransform(quat1, trans);

         transform.get(quatCheck, vec);

         assertEquals(quat1.x, quatCheck.x, 1e-3);
         assertEquals(quat1.y, quatCheck.y, 1e-3);
         assertEquals(quat1.z, quatCheck.z, 1e-3);
         assertEquals(quat1.w, quatCheck.w, 1e-3);
         assertEquals(vec.x, trans.x, 1e-3);
         assertEquals(vec.y, trans.y, 1e-3);
         assertEquals(vec.z, trans.z, 1e-3);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuat4d()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Vector3d trans = new Vector3d();
      Quat4d quatCheck = new Quat4d();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextDouble();
         quat1.y = random.nextDouble();
         quat1.z = random.nextDouble();
         quat1.w = random.nextDouble();
         quat1.normalize();

         RigidBodyTransform transform = new RigidBodyTransform(quat1,
               new Vector3d(0, 0, 0));

         transform.get(quatCheck);
         transform.getTranslation(trans);

         assertEquals(quat1.x, quatCheck.x, 1e-10);
         assertEquals(quat1.y, quatCheck.y, 1e-10);
         assertEquals(quat1.z, quatCheck.z, 1e-10);
         assertEquals(quat1.w, quatCheck.w, 1e-10);
         assertEquals(0, trans.x, 1e-10);
         assertEquals(0, trans.y, 1e-10);
         assertEquals(0, trans.z, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetTransformWithQuat4d()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Vector3d trans = new Vector3d();
      Quat4d quatCheck = new Quat4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextDouble();
         quat1.y = random.nextDouble();
         quat1.z = random.nextDouble();
         quat1.w = random.nextDouble();
         quat1.normalize();

         transform.setRotationAndZeroTranslation(quat1);

         transform.get(quatCheck);
         transform.getTranslation(trans);

         assertEquals(quat1.x, quatCheck.x, 1e-10);
         assertEquals(quat1.y, quatCheck.y, 1e-10);
         assertEquals(quat1.z, quatCheck.z, 1e-10);
         assertEquals(quat1.w, quatCheck.w, 1e-10);
         assertEquals(0, trans.x, 1e-10);
         assertEquals(0, trans.y, 1e-10);
         assertEquals(0, trans.z, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetTransformWithQuat4dAndVector3d()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Vector3d trans = new Vector3d();
      Vector3d vector = new Vector3d();
      Quat4d quatCheck = new Quat4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextDouble();
         quat1.y = random.nextDouble();
         quat1.z = random.nextDouble();
         quat1.w = random.nextDouble();
         quat1.normalize();

         randomizeVector(random, vector);

         transform.set(quat1, vector);

         transform.get(quatCheck);
         transform.getTranslation(trans);

         assertEquals(quat1.x, quatCheck.x, 1e-10);
         assertEquals(quat1.y, quatCheck.y, 1e-10);
         assertEquals(quat1.z, quatCheck.z, 1e-10);
         assertEquals(quat1.w, quatCheck.w, 1e-10);
         assertEquals(vector.x, trans.x, 1e-10);
         assertEquals(vector.y, trans.y, 1e-10);
         assertEquals(vector.z, trans.z, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetTransformWithQuat4f()
   {
      Random random = new Random();
      Quat4f quat1 = new Quat4f();
      Vector3f trans = new Vector3f();
      Quat4f quatCheck = new Quat4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextFloat();
         quat1.y = random.nextFloat();
         quat1.z = random.nextFloat();
         quat1.w = random.nextFloat();
         quat1.normalize();

         transform.setRotationAndZeroTranslation(quat1);

         transform.get(quatCheck);
         transform.getTranslation(trans);

         assertEquals(quat1.x, quatCheck.x, 1e-5);
         assertEquals(quat1.y, quatCheck.y, 1e-5);
         assertEquals(quat1.z, quatCheck.z, 1e-5);
         assertEquals(quat1.w, quatCheck.w, 1e-5);
         assertEquals(0, trans.x, 1e-5);
         assertEquals(0, trans.y, 1e-5);
         assertEquals(0, trans.z, 1e-5);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetTransformWithQuat4fAndVector3f()
   {
      Random random = new Random();
      Quat4f quat1 = new Quat4f();
      Vector3f trans = new Vector3f();
      Vector3f vector = new Vector3f();
      Quat4f quatCheck = new Quat4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextFloat();
         quat1.y = random.nextFloat();
         quat1.z = random.nextFloat();
         quat1.w = random.nextFloat();
         quat1.normalize();

         randomizeVector(random, vector);

         transform.set(quat1, vector);

         transform.get(quatCheck);
         transform.getTranslation(trans);

         assertEquals(quat1.x, quatCheck.x, 1e-5);
         assertEquals(quat1.y, quatCheck.y, 1e-5);
         assertEquals(quat1.z, quatCheck.z, 1e-5);
         assertEquals(quat1.w, quatCheck.w, 1e-5);
         assertEquals(vector.x, trans.x, 1e-5);
         assertEquals(vector.y, trans.y, 1e-5);
         assertEquals(vector.z, trans.z, 1e-5);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestDeterminant()
   {
      Random random = new Random();
      Matrix4d matrix = new Matrix4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);

         transform.set(matrix);
         assertEquals(matrix.determinant(), transform.determinant(), 1e-8);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuat4f()
   {
      Random random = new Random();
      Quat4f quat1 = new Quat4f();
      Vector3f trans = new Vector3f();
      Quat4f quatCheck = new Quat4f();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextFloat();
         quat1.y = random.nextFloat();
         quat1.z = random.nextFloat();
         quat1.w = random.nextFloat();
         quat1.normalize();

         RigidBodyTransform transform = new RigidBodyTransform(quat1,
               new Vector3f(0, 0, 0));

         transform.get(quatCheck, trans);

         assertEquals(quat1.x, quatCheck.x, 1e-3);
         assertEquals(quat1.y, quatCheck.y, 1e-3);
         assertEquals(quat1.z, quatCheck.z, 1e-3);
         assertEquals(quat1.w, quatCheck.w, 1e-3);
         assertEquals(0, trans.x, 1e-3);
         assertEquals(0, trans.y, 1e-3);
         assertEquals(0, trans.z, 1e-3);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuat4f2()
   {
      Random random = new Random();
      Quat4f quat1 = new Quat4f();
      Quat4f quatCheck = new Quat4f();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextFloat();
         quat1.y = random.nextFloat();
         quat1.z = random.nextFloat();
         quat1.w = random.nextFloat();
         quat1.normalize();

         RigidBodyTransform transform = new RigidBodyTransform(quat1,
               new Vector3f(0, 0, 0));

         transform.get(quatCheck);

         assertEquals(quat1.x, quatCheck.x, 1e-3);
         assertEquals(quat1.y, quatCheck.y, 1e-3);
         assertEquals(quat1.z, quatCheck.z, 1e-3);
         assertEquals(quat1.w, quatCheck.w, 1e-3);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuatWithZeroVectorElement()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Quat4d quatCheck = new Quat4d();
      Vector3d vec = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextDouble();
         quat1.y = random.nextDouble();
         quat1.z = 0;
         quat1.w = random.nextDouble();
         quat1.normalize();

         transform.setRotationAndZeroTranslation(quat1);

         transform.get(quatCheck, vec);

         assertEquals(quat1.x, quatCheck.x, 1e-7);
         assertEquals(quat1.y, quatCheck.y, 1e-7);
         assertEquals(quat1.z, quatCheck.z, 1e-7);
         assertEquals(quat1.w, quatCheck.w, 1e-7);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformWithQuatWithZeroScalarElement()
   {
      Random random = new Random();
      Quat4d quat1 = new Quat4d();
      Quat4d quatCheck = new Quat4d();
      Vector3d vec = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      quat1.x = random.nextDouble();
      quat1.y = random.nextDouble();
      quat1.z = random.nextDouble();
      quat1.w = 0;
      quat1.normalize();

      transform.setRotationAndZeroTranslation(quat1);

      transform.get(quatCheck, vec);

      assertEquals(quat1.x, quatCheck.x, 1e-7);
      assertEquals(quat1.y, quatCheck.y, 1e-7);
      assertEquals(quat1.z, quatCheck.z, 1e-7);
      assertEquals(quat1.w, quatCheck.w, 1e-7);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestGetTransformAsQuat4fAndVector3f()
   {
      Random random = new Random();
      Quat4f quat1 = new Quat4f();
      RigidBodyTransform transform = new RigidBodyTransform();
      Quat4f quatCheck = new Quat4f();
      Vector3f vec = new Vector3f();

      for (int i = 0; i < nTests; i++)
      {
         quat1.x = random.nextFloat();
         quat1.y = random.nextFloat();
         quat1.z = random.nextFloat();
         quat1.w = random.nextFloat();
         quat1.normalize();

         transform.setRotation(quat1);

         transform.get(quatCheck, vec);

         // Having trouble getting quat.w to be more precise than 1e-3 here.
         // It has to be floating point precision issue, but haven't figure
         // out how to fix that.
         assertEquals(quat1.x, quatCheck.x, 1e-3);
         assertEquals(quat1.y, quatCheck.y, 1e-3);
         assertEquals(quat1.z, quatCheck.z, 1e-3);
         assertEquals(quat1.w, quatCheck.w, 1e-3);
      }
   }

   @EstimatedDuration(duration = 0.0)
   @Test(timeout = 30000)
   public void TestGetRotationAxisAngle4d()
   {
      double epsilonAssert = 1e-6;
      
      Random random = new Random(100L);
      
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      
      Vector3d axisActual = RandomTools.generateRandomVector(random);
      axisActual.scale(1.0 / axisActual.length());
      
      Vector3d axisComputed = new Vector3d();

      AxisAngle4d axisAngleActual = new AxisAngle4d();
      AxisAngle4d axisAngleToPack = new AxisAngle4d();

      double[] anglesToTest;
      
      int numberOfPassedTests = 0;
      
      for (int i=0; i<2; i++)
      {
         if (i==0)
         {
            anglesToTest = AngleTools.generateArrayOfTestAngles(100, 1e-5, false, false);
         }

         else
         {
            anglesToTest = new double[]{-3.0*Math.PI, -2.0*Math.PI, -Math.PI, -0.5*Math.PI, 0.0, 0.5*Math.PI, Math.PI, 2.0*Math.PI, 3.0*Math.PI};
         }
         
         for (double angleActual : anglesToTest)
         {
            axisAngleActual.set(axisActual, angleActual);
            rotationTransform.setRotationAndZeroTranslation(axisAngleActual);

            rotationTransform.getRotation(axisAngleToPack);
            
            double angleComputed = axisAngleToPack.getAngle();
            axisComputed.set(axisAngleToPack.getX(), axisAngleToPack.getY(), axisAngleToPack.getZ());
            
            double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(angleActual, angleComputed);

            if ( Math.abs(angleDifference) > epsilonAssert)
            {
               boolean rotationAxisIsFlipped = MathTools.epsilonEquals(axisComputed.dot(axisActual), -1.0, epsilonAssert);
               
               String errorMsg = "Computed rotation angle, " + Math.toDegrees(angleComputed) + " degrees, does not equal actual rotation angle, "
                     + Math.toDegrees(angleActual) + "degrees \n and computed rotation axis: " + axisComputed + "\n is not a flipped version of actual axis: "
                     + axisActual + "\n Number of Passed Tests: " + numberOfPassedTests + "\n";

               assertTrue(errorMsg, rotationAxisIsFlipped);
            }
            numberOfPassedTests++;
         } 
      }

      
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateTransformFromMatrix3dAndGetTransformAsMatrix3d()
   {
      Random random = new Random();
      Matrix3d matrix = new Matrix3d();
      Vector3d vector = new Vector3d();
      Matrix3d matrixCheck = new Matrix3d();
      Vector3d vectorCheck = new Vector3d();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);

         transform.getRotation(matrixCheck);
         transform.getTranslation(vectorCheck);

         JUnitTools
               .assertMatrix3dEquals("", matrixCheck, matrixCheck, 1e-20);
         JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-20);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetMatrixScaleWithMatrix3d()
   {
      Random random = new Random();
      Matrix3d matrix = new Matrix3d();
      Vector3d vector = new Vector3d();
      Matrix3d matrixCheck = new Matrix3d();
      Vector3d vectorCheck = new Vector3d();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         randomizeVector(random, vector);
         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);

         double scale = random.nextDouble();
         matrix.mul(scale);

         transform.setRotation(matrix);

         transform.getRotation(matrixCheck);
         transform.getTranslation(vectorCheck);

         JUnitTools.assertMatrix3dEquals("", matrix, matrixCheck, 1e-8);
         JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-8);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetMatrixScaleWithMatrix3f()
   {
      Random random = new Random();
      Matrix3f matrix = new Matrix3f();
      Vector3f vector = new Vector3f();
      Matrix3f matrixCheck = new Matrix3f();
      Vector3f vectorCheck = new Vector3f();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);

         double scale = random.nextDouble();
         matrix.mul((float) scale);

         transform.setRotation(matrix);

         transform.getRotation(matrixCheck);
         transform.getTranslation(vectorCheck);

         JUnitTools.assertMatrix3fEquals("", matrix, matrixCheck, 1e-3);
         JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-3);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix3d()
   {
      Random random = new Random();
      Matrix3d matrix = new Matrix3d();
      Matrix4d matrix4 = new Matrix4d();
      Matrix4d matrixCheck = new Matrix4d();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         matrix4.m00 = matrix.m00;
         matrix4.m01 = matrix.m01;
         matrix4.m02 = matrix.m02;
         matrix4.m10 = matrix.m10;
         matrix4.m11 = matrix.m11;
         matrix4.m12 = matrix.m12;
         matrix4.m20 = matrix.m20;
         matrix4.m21 = matrix.m21;
         matrix4.m22 = matrix.m22;
         matrix4.m33 = 1;

         RigidBodyTransform transform = new RigidBodyTransform(matrix, new Vector3d(0, 0,
               0));
         transform.get(matrixCheck);

         JUnitTools.assertMatrix4dEquals("", matrixCheck, matrix4, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetAsMatrix3d()
   {
      Random random = new Random();
      Matrix3d matrix = new Matrix3d();
      Matrix4d matrix4 = new Matrix4d();
      Matrix4d matrixCheck = new Matrix4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         matrix4.m00 = matrix.m00;
         matrix4.m01 = matrix.m01;
         matrix4.m02 = matrix.m02;
         matrix4.m03 = 0;
         matrix4.m10 = matrix.m10;
         matrix4.m11 = matrix.m11;
         matrix4.m12 = matrix.m12;
         matrix4.m13 = 0;
         matrix4.m20 = matrix.m20;
         matrix4.m21 = matrix.m21;
         matrix4.m22 = matrix.m22;
         matrix4.m23 = 0;
         matrix4.m30 = 0;
         matrix4.m31 = 0;
         matrix4.m32 = 0;
         matrix4.m33 = 1;

         transform.setRotationAndZeroTranslation(matrix);

         transform.get(matrixCheck);

         JUnitTools.assertMatrix4dEquals("", matrixCheck, matrix4, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetAsVector3d()
   {
      Random random = new Random();
      Vector3d vector = new Vector3d();
      Matrix4d matrix4 = new Matrix4d();
      Matrix4d matrixCheck = new Matrix4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         randomizeVector(random, vector);

         transform.setTranslationAndIdentityRotation(vector);
         matrix4.m00 = 1;
         matrix4.m11 = 1;
         matrix4.m22 = 1;
         matrix4.m33 = 1;
         matrix4.m03 = vector.x;
         matrix4.m13 = vector.y;
         matrix4.m23 = vector.z;

         transform.get(matrixCheck);

         JUnitTools.assertMatrix4dEquals("", matrixCheck, matrix4, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetAsVector3f()
   {
      Random random = new Random();
      Vector3f vector = new Vector3f();
      Matrix4f matrix4 = new Matrix4f();
      Matrix4f matrixCheck = new Matrix4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         randomizeVector(random, vector);

         transform.setTranslationAndIdentityRotation(vector);
         matrix4.m00 = 1;
         matrix4.m11 = 1;
         matrix4.m22 = 1;
         matrix4.m33 = 1;
         matrix4.m03 = vector.x;
         matrix4.m13 = vector.y;
         matrix4.m23 = vector.z;

         transform.get(matrixCheck);

         JUnitTools.assertMatrix4fEquals("", matrixCheck, matrix4, 1e-5);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestSetAsMatrix3f()
   {
      Random random = new Random();
      Matrix3f matrix = new Matrix3f();
      Matrix4f matrix4 = new Matrix4f();
      Matrix4f matrixCheck = new Matrix4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         matrix4.m00 = matrix.m00;
         matrix4.m01 = matrix.m01;
         matrix4.m02 = matrix.m02;
         matrix4.m03 = 0;
         matrix4.m10 = matrix.m10;
         matrix4.m11 = matrix.m11;
         matrix4.m12 = matrix.m12;
         matrix4.m13 = 0;
         matrix4.m20 = matrix.m20;
         matrix4.m21 = matrix.m21;
         matrix4.m22 = matrix.m22;
         matrix4.m23 = 0;
         matrix4.m30 = 0;
         matrix4.m31 = 0;
         matrix4.m32 = 0;
         matrix4.m33 = 1;

         transform.setRotationAndZeroTranslation(matrix);

         transform.get(matrixCheck);

         JUnitTools.assertMatrix4fEquals("", matrixCheck, matrix4, 1e-5);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix3d2()
   {
      Random random = new Random();
      Matrix3d matrix = new Matrix3d();
      Vector3d vector = new Vector3d();
      Vector3d vectorCheck = new Vector3d();
      Matrix3d matrixCheck = new Matrix3d();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);
         transform.normalize();
         transform.get(matrixCheck, vectorCheck);

         JUnitTools.assertMatrix3dEquals("", matrixCheck, matrix, 1e-12);
         JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-12);

         transform.get(matrixCheck);

         JUnitTools.assertMatrix3dEquals("", matrixCheck, matrix, 1e-12);

         transform.get(vectorCheck);

         JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix3f()
   {
      Random random = new Random();
      Matrix3f matrix = new Matrix3f();
      Matrix4f matrix4 = new Matrix4f();
      Matrix4f matrixCheck = new Matrix4f();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         matrix4.m00 = matrix.m00;
         matrix4.m01 = matrix.m01;
         matrix4.m02 = matrix.m02;
         matrix4.m10 = matrix.m10;
         matrix4.m11 = matrix.m11;
         matrix4.m12 = matrix.m12;
         matrix4.m20 = matrix.m20;
         matrix4.m21 = matrix.m21;
         matrix4.m22 = matrix.m22;
         matrix4.m33 = 1;

         RigidBodyTransform transform = new RigidBodyTransform(matrix, new Vector3f(0, 0,
               0));
         transform.get(matrixCheck);

         JUnitTools.assertMatrix4fEquals("", matrixCheck, matrix4, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix3f2()
   {
      Random random = new Random();
      Matrix3f matrix = new Matrix3f();
      Vector3f vector = new Vector3f();
      Vector3f vectorCheck = new Vector3f();
      Matrix3f matrixCheck = new Matrix3f();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);

         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);
         transform.get(matrixCheck, vectorCheck);

         JUnitTools.assertMatrix3fEquals("", matrixCheck, matrix, 1e-6);
         JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-6);

         transform.get(matrixCheck);
         JUnitTools.assertMatrix3fEquals("", matrixCheck, matrix, 1e-6);

         transform.get(vectorCheck);
         JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromTransform()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F matrixCheck = new DenseMatrix64F(4, 4);

      createRandomTransformationMatrix(matrix, random);
      RigidBodyTransform transform = new RigidBodyTransform(matrix);
      RigidBodyTransform transformCheck = new RigidBodyTransform(transform);

      transform.get(matrix);
      transformCheck.get(matrixCheck);

      JUnitTools.assertMatrixEquals("", matrixCheck, matrix, 1e-12);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix3fAndVector3f()
   {
      Random random = new Random();
      Matrix3f matrix = new Matrix3f();
      Vector3f vector = new Vector3f();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);
         Matrix3f matrixCheck = new Matrix3f();
         Vector3f vectorCheck = new Vector3f();

         transform.getRotation(matrixCheck);
         transform.getTranslation(vectorCheck);

         JUnitTools
               .assertMatrix3fEquals("", matrixCheck, matrixCheck, 1e-20);
         JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-20);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateIdentityTransform()
   {
      for (int j = 0; j < nTests; j++)
      {
         double[] identityMatrix4By4 = new double[16];

         identityMatrix4By4[0] = 1;
         identityMatrix4By4[5] = 1;
         identityMatrix4By4[10] = 1;
         identityMatrix4By4[15] = 1;

         RigidBodyTransform transform3d = new RigidBodyTransform();

         double error = 0;
         ;
         double[] doubleTransform = new double[16];
         transform3d.get(doubleTransform);
         for (int i = 0; i < 16; i++)
         {
            error += (identityMatrix4By4[i] - doubleTransform[i]);
         }

         assertTrue(error == 0);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix4d()
   {
      Matrix4d matrix = new Matrix4d();
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(matrix);

         Matrix4d check = new Matrix4d();

         transform3d.get(check);

         JUnitTools.assertMatrix4dEquals("", check, matrix, 1e-20);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromColumnMajorMatrix4d()
   {
      Matrix4d matrix = new Matrix4d();
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         matrix.transpose();
         RigidBodyTransform transform3d = new RigidBodyTransform();
         transform3d.setAsTranspose(matrix);
         matrix.transpose();

         Matrix4d check = new Matrix4d();

         transform3d.get(check);

         JUnitTools.assertMatrix4dEquals("", check, matrix, 1e-20);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromColumnMajorMatrix4()
   {
      Matrix4f matrix = new Matrix4f();
      Random random = new Random();
      float[] columnMajorTransformationMatrix = new float[16];

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         matrix.transpose();
         createFloatArrayFromMatrix4d(columnMajorTransformationMatrix, matrix);
         RigidBodyTransform transform3d = new RigidBodyTransform();

         transform3d.setAsTranspose(columnMajorTransformationMatrix);

         Matrix4f check = new Matrix4f();

         transform3d.get(check);

         matrix.transpose();
         JUnitTools.assertMatrix4fEquals("", check, matrix, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromColumnMajorMatrix4f()
   {
      Matrix4f matrix = new Matrix4f();
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         matrix.transpose();
         RigidBodyTransform transform3d = new RigidBodyTransform();
         transform3d.setAsTranspose(matrix);
         matrix.transpose();

         Matrix4f check = new Matrix4f();

         transform3d.get(check);

         JUnitTools.assertMatrix4fEquals("", check, matrix, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromMatrix4f()
   {
      Matrix4f matrix = new Matrix4f();
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(matrix);

         Matrix4f check = new Matrix4f();

         transform3d.get(check);

         JUnitTools.assertMatrix4fEquals("", check, matrix, 1e-8);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromDenseMatrix()
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(denseMatrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(denseMatrix);

         DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);

         transform3d.get(checkMatrix);

         JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromDenseMatrixAndVector3d()
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
      Vector3d trans = new Vector3d();
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(denseMatrix, random);
         randomizeVector(random, trans);

         RigidBodyTransform transform3d = new RigidBodyTransform(denseMatrix, trans);

         DenseMatrix64F checkMatrix = new DenseMatrix64F(3, 3);
         Vector3d checkVector = new Vector3d();

         transform3d.getRotation(checkMatrix);
         transform3d.getTranslation(checkVector);

         JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromDenseMatrixThatIsTheWrongSize()
   {

      try
      {
         RigidBodyTransform transform3d = new RigidBodyTransform(new DenseMatrix64F(9, 9),
               new Vector3d(0, 0, 0));
      }
      catch (Exception e)
      {
         // If the code gets here, it passed the test.
         assertTrue(true);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromDoubleArray()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      double[] doubleArrayCheck = new double[16];

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);

         double[] matrixAsDoubleArray = matrix.getData();
         RigidBodyTransform transform = new RigidBodyTransform(matrixAsDoubleArray);

         transform.get(doubleArrayCheck);
         assertArrayEquals(matrixAsDoubleArray, doubleArrayCheck, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestCreateFromfloatArray()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      float[] floatArray = new float[16];

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);

         double[] matrixAsFloatArray = matrix.getData();
         float[] floatArrayCheck = putDoublesInFloatArray(matrixAsFloatArray);

         RigidBodyTransform transform = new RigidBodyTransform(floatArrayCheck);

         transform.get(floatArray);

         assertArrayEquals(floatArray, floatArrayCheck, (float) 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformMultiplication()
   {
      Random random = new Random();
      Matrix4d matrix1 = new Matrix4d();
      Matrix4d matrix2 = new Matrix4d();

      for (int i = 0; i < 1; i++)
      {
         createRandomTransformationMatrix(matrix1, random);
         createRandomTransformationMatrix(matrix2, random);

         RigidBodyTransform transform1 = new RigidBodyTransform(matrix1);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix2);

         matrix1.mul(matrix2);
         transform1.multiply(transform2);

         Matrix4d transformMat4d = new Matrix4d();

         transform1.get(transformMat4d);

         JUnitTools.assertMatrix4dEquals("", matrix1, transformMat4d, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformMultiplication2()
   {
      Random random = new Random();
      Matrix4d matrix1 = new Matrix4d();
      Matrix4d matrix2 = new Matrix4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < 1; i++)
      {
         createRandomTransformationMatrix(matrix1, random);
         createRandomTransformationMatrix(matrix2, random);

         RigidBodyTransform transform1 = new RigidBodyTransform(matrix1);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix2);

         matrix1.mul(matrix2);
         transform.multiply(transform1, transform2);

         Matrix4d transformMat4d = new Matrix4d();

         transform.get(transformMat4d);

         JUnitTools.assertMatrix4dEquals("", matrix1, transformMat4d, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformMultiplication3()
   {
      Random random = new Random();
      Matrix4d matrix1 = new Matrix4d();
      Matrix4d matrix2 = new Matrix4d();

      for (int i = 0; i < 1; i++)
      {
         createRandomTransformationMatrix(matrix1, random);
         createRandomTransformationMatrix(matrix2, random);

         RigidBodyTransform transform1 = new RigidBodyTransform(matrix1);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix2);
         // matrix1.mul(matrix2);
         matrix2.mul(matrix1);
         transform1.multiply(transform2, transform1);

         Matrix4d transform1Mat4d = new Matrix4d();

         transform1.get(transform1Mat4d);

         JUnitTools
               .assertMatrix4dEquals("", matrix2, transform1Mat4d, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformMultiplication4()
   {
      Random random = new Random();
      Matrix4d matrix1 = new Matrix4d();
      Matrix4d matrix2 = new Matrix4d();

      for (int i = 0; i < 1; i++)
      {
         createRandomTransformationMatrix(matrix1, random);
         createRandomTransformationMatrix(matrix2, random);

         RigidBodyTransform transform1 = new RigidBodyTransform(matrix1);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix2);

         matrix2.mul(matrix2);
         transform2.multiply(transform2);

         Matrix4d transform1Mat4d = new Matrix4d();

         transform2.get(transform1Mat4d);

         JUnitTools
               .assertMatrix4dEquals("", matrix2, transform1Mat4d, 1e-15);
      }
   }

	@EstimatedDuration(duration = 1.1)
	@Test(timeout = 30000)
   public void TestOrthogonalityOfChainOfTransformations()
   {
      Random random = new Random();
      Matrix4d matrix1 = new Matrix4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < 10000; i++)
         {
            createRandomTransformationMatrix(matrix1, random);
            RigidBodyTransform transformToMult = new RigidBodyTransform(matrix1);
            transform.multiply(transformToMult);
         }

         Matrix4d retXform = new Matrix4d();
         transform.get(retXform);

         Matrix3d mat = new Matrix3d();
         transform.getRotation(mat);
         RotationFunctions.isRotationProper(mat);
         assertTrue(checkOrthogonality(transform));
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestMatrixInverse()
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(denseMatrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(denseMatrix);

         CommonOps.invert(denseMatrix);

         transform3d.invert();
         transform3d.get(checkMatrix);

         JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestMatrixInverse7()
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
      Random random = new Random();
      RigidBodyTransform transform2 = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(denseMatrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(denseMatrix);
         transform2.invert(transform3d);
         transform3d.invert();
         CommonOps.invert(denseMatrix);

         transform3d.get(checkMatrix);
         transform2.get(denseMatrix);

         JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestMatrixManyMatrixInverses()
   {
      Matrix4d matrix = new Matrix4d();
      Matrix4d checkMatrix = new Matrix4d();
      Matrix3d mat = new Matrix3d();
      Random random = new Random();
      int nInverses = 100;
      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(matrix);

         for (int j = 0; j < nInverses; j++)
         {
            transform3d.invert();
            matrix.invert();
         }
         transform3d.get(checkMatrix);
         transform3d.getRotation(mat);
         RotationFunctions.isRotationProper(mat);

         JUnitTools.assertMatrix4dEquals("", matrix, checkMatrix, 1e-10);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestInvertTransform2()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
      DenseMatrix64F matrix2 = new DenseMatrix64F(4, 4);
      Vector3d vector = new Vector3d();
      DenseMatrix64F checkMatrix2 = new DenseMatrix64F(4, 4);
      DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);
         transform.get(matrix2);
         CommonOps.invert(matrix2);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix2);

         transform2.invert();

         transform.get(checkMatrix);
         transform2.get(checkMatrix2);

         JUnitTools.assertMatrixEquals(checkMatrix2, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestInvertTransform3()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
      DenseMatrix64F matrix2 = new DenseMatrix64F(4, 4);
      Vector3d vector = new Vector3d();
      DenseMatrix64F checkMatrix2 = new DenseMatrix64F(4, 4);
      DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
      Random random = new Random();

      for (int i = 0; i < nTests; i++)
      {
         createRandomRotationMatrix(matrix, random);
         randomizeVector(random, vector);

         RigidBodyTransform transform = new RigidBodyTransform(matrix, vector);
         transform.invert();
         transform.get(matrix2);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix2);

         transform.get(checkMatrix);
         transform2.get(checkMatrix2);

         JUnitTools.assertMatrixEquals(checkMatrix2, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestMatrixInverse2()
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
      Matrix4d matrix = new Matrix4d();
      Random random = new Random();
      RigidBodyTransform transform2 = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(denseMatrix, random);
         createRandomTransformationMatrix(matrix, random);

         RigidBodyTransform transform3d = new RigidBodyTransform(matrix);
         transform2.invert(transform3d);

         transform2.invert();

         matrix.invert();
         transform2.get(denseMatrix);
         transform3d.get(checkMatrix);

         JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestEpsilonEquals()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      double epsilon = 1e-8;

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         RigidBodyTransform transform1 = new RigidBodyTransform(matrix);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix);

         assertTrue(transform1.epsilonEquals(transform2, epsilon));
         assertTrue(transform2.epsilonEquals(transform1, epsilon));

         for (int row = 0; row < 4; row++)
         {
            for (int col = 0; col < 4; col++)
            {
               // Change one element at a time.
               matrix.set(row, col, 2 + 10 * random.nextDouble());
               transform1.set(matrix);
               assertFalse(transform1.epsilonEquals(transform2, epsilon));
               transform1.set(transform2);
            }
         }
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestRotX()
   {
      Vector3d vector = new Vector3d(1, 0, 0);
      Vector3d vector2 = new Vector3d(0, 1, 0);
      Vector3d vector3 = new Vector3d(0, 0, 1);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.rotX(Math.PI / 2);
      transform.getRotation(rotationMatrix);

      MatrixTools.mult(rotationMatrix, vector);
      MatrixTools.mult(rotationMatrix, vector2);
      MatrixTools.mult(rotationMatrix, vector3);

      JUnitTools.assertVector3dEquals("", new Vector3d(1, 0, 0), vector,
            1e-12);
      JUnitTools.assertVector3dEquals("", new Vector3d(0, 0, 1), vector2,
            1e-12);
      JUnitTools.assertVector3dEquals("", new Vector3d(0, -1, 0), vector3,
            1e-12);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestRotY()
   {
      Vector3d vector = new Vector3d(1, 0, 0);
      Vector3d vector2 = new Vector3d(0, 1, 0);
      Vector3d vector3 = new Vector3d(0, 0, 1);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.rotY(Math.PI / 2);
      transform.getRotation(rotationMatrix);

      MatrixTools.mult(rotationMatrix, vector);
      MatrixTools.mult(rotationMatrix, vector2);
      MatrixTools.mult(rotationMatrix, vector3);

      JUnitTools.assertVector3dEquals("", new Vector3d(0, 0, -1), vector,
            1e-12);
      JUnitTools.assertVector3dEquals("", new Vector3d(0, 1, 0), vector2,
            1e-12);
      JUnitTools.assertVector3dEquals("", new Vector3d(1, 0, 0), vector3,
            1e-12);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestRotZ()
   {
      Vector3d vector = new Vector3d(1, 0, 0);
      Vector3d vector2 = new Vector3d(0, 1, 0);
      Vector3d vector3 = new Vector3d(0, 0, 1);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.rotZ(Math.PI / 2);
      transform.getRotation(rotationMatrix);

      MatrixTools.mult(rotationMatrix, vector);
      MatrixTools.mult(rotationMatrix, vector2);
      MatrixTools.mult(rotationMatrix, vector3);

      JUnitTools.assertVector3dEquals("", new Vector3d(0, 1, 0), vector,
            1e-12);
      JUnitTools.assertVector3dEquals("", new Vector3d(-1, 0, 0), vector2,
            1e-12);
      JUnitTools.assertVector3dEquals("", new Vector3d(0, 0, 1), vector3,
            1e-12);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestEquals()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         RigidBodyTransform transform1 = new RigidBodyTransform(matrix);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix);

         assertTrue(transform1.equals(transform2));

         transform1.mat10 = transform1.mat10 + 1;
         assertFalse(transform1.equals(transform2));
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestEqualsWithScale()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         RigidBodyTransform transform1 = new RigidBodyTransform(matrix);
         RigidBodyTransform transform2 = new RigidBodyTransform(matrix);

         assertTrue(transform1.equals(transform2));

         transform1.mat10 = transform1.mat10 + 1;
         assertFalse(transform1.equals(transform2));
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestEulerAngles()
   {
      Random random = new Random();
      Vector3d vector = new Vector3d();
      Vector3d vectorToCheck = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();
      for (int i = 0; i < nTests; i++)
      {
         vector.x = -Math.PI + random.nextDouble() * 2 * Math.PI;
         // Inverse Euler conversion is not defined for vector.y = 0 and the
         // solution switches for
         // vector.y outside of -pi/2 to pi/2. Separate tests can be created
         // if the whole range wants to be
         // tested. The inverse Euler transform is only used for testing
         // purposes, so it is not an issue.
         vector.y = -(Math.PI / 2 - 0.01) * random.nextDouble() - 0.01;
         vector.z = -Math.PI + random.nextDouble() * 2 * Math.PI;

         transform.setEuler(vector);

         transform.getEulerXYZ(vectorToCheck);
         JUnitTools.assertVector3dEquals("", vector, vectorToCheck, 1e-5);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector4d()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      Vector4d vector = new Vector4d();
      Vector4d vector2 = new Vector4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);

         createRandomTransform4Vector(vector, random);
         vector2.set(vector);

         MatrixTools.mult(matrix, vector);
         transform.transform(vector2);

         JUnitTools.assertVector4dEquals("", vector, vector2, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector4d2()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      Vector4d vector = new Vector4d();
      Vector4d vector2 = new Vector4d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);

         createRandomTransform4Vector(vector, random);

         transform.transform(vector, vector2);
         MatrixTools.mult(matrix, vector);

         JUnitTools.assertVector4dEquals("", vector, vector2, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector4f()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      Vector4f vector = new Vector4f();
      Vector4f vector2 = new Vector4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);

         createRandomTransform4Vector(vector, random);
         vector2.set(vector);

         MatrixTools.mult(matrix, vector);
         transform.transform(vector2);

         JUnitTools.assertVector4fEquals("", vector, vector2, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector4f2()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      Vector4f vector = new Vector4f();
      Vector4f vector2 = new Vector4f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);

         createRandomTransform4Vector(vector, random);

         transform.transform(vector, vector2);
         MatrixTools.mult(matrix, vector);

         JUnitTools.assertVector4fEquals("", vector, vector2, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector3d()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Vector3d vector = new Vector3d();
      Vector3d vector2 = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);

         randomizeVector(random, vector);
         vector2.set(vector);

         MatrixTools.mult(rotationMatrix, vector);
         transform.transform(vector2);

         JUnitTools.assertVector3dEquals("", vector, vector2, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector3f()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Vector3f vector = new Vector3f();
      Vector3f vector2 = new Vector3f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);

         randomizeVector(random, vector);
         vector2.set(vector);

         MatrixTools.mult(rotationMatrix, vector);
         transform.transform(vector2);

         JUnitTools.assertVector3fEquals("", vector, vector2, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector3f2()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Vector3f vector = new Vector3f();
      Vector3f vector2 = new Vector3f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);

         randomizeVector(random, vector);

         transform.transform(vector, vector2);
         MatrixTools.mult(rotationMatrix, vector);

         JUnitTools.assertVector3fEquals("", vector, vector2, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformVector3d2()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Vector3d vector = new Vector3d();
      Vector3d vector2 = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);

         randomizeVector(random, vector);

         transform.transform(vector, vector2);
         MatrixTools.mult(rotationMatrix, vector);

         JUnitTools.assertVector3dEquals("", vector, vector2, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformPoint3d()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Point3d point = new Point3d();
      Point3d point2 = new Point3d();
      Vector3d vector = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);
         transform.getTranslation(vector);

         randomizePoint3d(random, point);
         point2.set(point);

         MatrixTools.mult(rotationMatrix, point);
         point.add(vector);
         transform.transform(point2);

         JUnitTools.assertPoint3dEquals("", point, point2, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformPoint3d2()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Point3d point = new Point3d();
      Point3d point2 = new Point3d();
      Vector3d vector = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);
         transform.getTranslation(vector);

         randomizePoint3d(random, point);

         transform.transform(point, point2);
         MatrixTools.mult(rotationMatrix, point);
         point.add(vector);

         JUnitTools.assertPoint3dEquals("", point, point2, 1e-12);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformPoint3f()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Point3f point = new Point3f();
      Point3f point2 = new Point3f();
      Vector3f vector = new Vector3f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);
         transform.getTranslation(vector);

         randomizePoint3f(random, point);
         point2.set(point);

         MatrixTools.mult(rotationMatrix, point);
         point.add(vector);
         transform.transform(point2);

         JUnitTools.assertPoint3fEquals("", point, point2, 1e-6);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestTransformPoint3f2()
   {
      Random random = new Random();
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
      Point3f point = new Point3f();
      Point3f point2 = new Point3f();
      Vector3f vector = new Vector3f();
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < nTests; i++)
      {
         createRandomTransformationMatrix(matrix, random);
         transform.set(matrix);
         transform.getRotation(rotationMatrix);
         transform.getTranslation(vector);

         randomizePoint3f(random, point);

         transform.transform(point, point2);
         MatrixTools.mult(rotationMatrix, point);
         point.add(vector);

         JUnitTools.assertPoint3fEquals("", point, point2, 1e-6);
      }
   }
   
   private boolean checkOrthogonality(RigidBodyTransform transform)
   {
      Matrix3d matrix = new Matrix3d();
      transform.getRotation(matrix);

      Vector3d tmpVecX = new Vector3d(matrix.m00, matrix.m10, matrix.m20);
      Vector3d tmpVecY = new Vector3d(matrix.m01, matrix.m11, matrix.m21);
      Vector3d tmpVecZ = new Vector3d(matrix.m02, matrix.m12, matrix.m22);

      return (tmpVecX.lengthSquared() - 1 < 1e-8)
            && (tmpVecY.lengthSquared() - 1 < 1e-8)
            && (tmpVecZ.lengthSquared() - 1 < 1e-8);
   }

   private void randomizeVector(Random random, Vector3d vector)
   {
      vector.x = random.nextDouble();
      vector.y = random.nextDouble();
      vector.z = random.nextDouble();
   }

   private void randomizeVector(Random random, Vector3f vector)
   {
      vector.x = random.nextFloat();
      vector.y = random.nextFloat();
      vector.z = random.nextFloat();
   }

   private void randomizePoint3d(Random random, Point3d point)
   {
      point.x = random.nextDouble();
      point.y = random.nextDouble();
      point.z = random.nextDouble();
   }

   private void randomizePoint3f(Random random, Point3f point)
   {
      point.x = random.nextFloat();
      point.y = random.nextFloat();
      point.z = random.nextFloat();
   }

   private void createRandomTransformationMatrix(DenseMatrix64F matrix,
         Random random)
   {
      Matrix3d rotX = new Matrix3d();
      Matrix3d rotY = new Matrix3d();
      Matrix3d rotZ = new Matrix3d();
      Vector3d trans = new Vector3d();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.set(0, 0, rotX.m00);
      matrix.set(0, 1, rotX.m01);
      matrix.set(0, 2, rotX.m02);
      matrix.set(0, 3, trans.x);
      matrix.set(1, 0, rotX.m10);
      matrix.set(1, 1, rotX.m11);
      matrix.set(1, 2, rotX.m12);
      matrix.set(1, 3, trans.y);
      matrix.set(2, 0, rotX.m20);
      matrix.set(2, 1, rotX.m21);
      matrix.set(2, 2, rotX.m22);
      matrix.set(2, 3, trans.z);
      matrix.set(3, 0, 0);
      matrix.set(3, 1, 0);
      matrix.set(3, 2, 0);
      matrix.set(3, 3, 1);
   }

   private void createRandomRotationMatrix(Matrix3d matrix, Random random)
   {
      Matrix3d rotX = new Matrix3d();
      Matrix3d rotY = new Matrix3d();
      Matrix3d rotZ = new Matrix3d();
      Vector3d trans = new Vector3d();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.m00 = rotX.m00;
      matrix.m01 = rotX.m01;
      matrix.m02 = rotX.m02;
      matrix.m10 = rotX.m10;
      matrix.m11 = rotX.m11;
      matrix.m12 = rotX.m12;
      matrix.m20 = rotX.m20;
      matrix.m21 = rotX.m21;
      matrix.m22 = rotX.m22;
   }

   private void createRandomRotationMatrix(DenseMatrix64F matrix, Random random)
   {
      Matrix3d rotX = new Matrix3d();
      Matrix3d rotY = new Matrix3d();
      Matrix3d rotZ = new Matrix3d();
      Vector3d trans = new Vector3d();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.set(0, 0, rotX.m00);
      matrix.set(0, 1, rotX.m01);
      matrix.set(0, 2, rotX.m02);
      matrix.set(1, 0, rotX.m10);
      matrix.set(1, 1, rotX.m11);
      matrix.set(1, 2, rotX.m12);
      matrix.set(2, 0, rotX.m20);
      matrix.set(2, 1, rotX.m21);
      matrix.set(2, 2, rotX.m22);
   }

   private void createRandomRotationMatrix(Matrix3f matrix, Random random)
   {
      Matrix3f rotX = new Matrix3f();
      Matrix3f rotY = new Matrix3f();
      Matrix3f rotZ = new Matrix3f();
      Vector3f trans = new Vector3f();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.m00 = rotX.m00;
      matrix.m01 = rotX.m01;
      matrix.m02 = rotX.m02;
      matrix.m10 = rotX.m10;
      matrix.m11 = rotX.m11;
      matrix.m12 = rotX.m12;
      matrix.m20 = rotX.m20;
      matrix.m21 = rotX.m21;
      matrix.m22 = rotX.m22;
   }

   private void createRandomTransformationMatrix(Matrix4d matrix, Random random)
   {
      Matrix3d rotX = new Matrix3d();
      Matrix3d rotY = new Matrix3d();
      Matrix3d rotZ = new Matrix3d();
      Vector3d trans = new Vector3d();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.m00 = rotX.m00;
      matrix.m01 = rotX.m01;
      matrix.m02 = rotX.m02;
      matrix.m03 = trans.x;
      matrix.m10 = rotX.m10;
      matrix.m11 = rotX.m11;
      matrix.m12 = rotX.m12;
      matrix.m13 = trans.y;
      matrix.m20 = rotX.m20;
      matrix.m21 = rotX.m21;
      matrix.m22 = rotX.m22;
      matrix.m23 = trans.z;
      matrix.m30 = 0;
      matrix.m31 = 0;
      matrix.m32 = 0;
      matrix.m33 = 1;
   }

   private void createRandomTransformationMatrix(Matrix4f matrix, Random random)
   {
      Matrix3d rotX = new Matrix3d();
      Matrix3d rotY = new Matrix3d();
      Matrix3d rotZ = new Matrix3d();
      Vector3d trans = new Vector3d();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.m00 = (float) rotX.m00;
      matrix.m01 = (float) rotX.m01;
      matrix.m02 = (float) rotX.m02;
      matrix.m03 = (float) trans.x;
      matrix.m10 = (float) rotX.m10;
      matrix.m11 = (float) rotX.m11;
      matrix.m12 = (float) rotX.m12;
      matrix.m13 = (float) trans.y;
      matrix.m20 = (float) rotX.m20;
      matrix.m21 = (float) rotX.m21;
      matrix.m22 = (float) rotX.m22;
      matrix.m23 = (float) trans.z;
      matrix.m30 = 0;
      matrix.m31 = 0;
      matrix.m32 = 0;
      matrix.m33 = 1;
   }

   private void createRandomRotationMatrixX(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = 1;
      matrix.m01 = 0;
      matrix.m02 = 0;
      matrix.m10 = 0;
      matrix.m11 = cTheta;
      matrix.m12 = -sTheta;
      matrix.m20 = 0;
      matrix.m21 = sTheta;
      matrix.m22 = cTheta;
   }

   private void createRandomRotationMatrixX(Random random, Matrix3f matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = 1;
      matrix.m01 = 0;
      matrix.m02 = 0;
      matrix.m10 = 0;
      matrix.m11 = (float) cTheta;
      matrix.m12 = (float) -sTheta;
      matrix.m20 = 0;
      matrix.m21 = (float) sTheta;
      matrix.m22 = (float) cTheta;
   }

   private void createRandomRotationMatrixY(Random random, Matrix3f matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = (float) cTheta;
      matrix.m01 = 0;
      matrix.m02 = (float) sTheta;
      matrix.m10 = 0;
      matrix.m11 = 1;
      matrix.m12 = 0;
      matrix.m20 = (float) -sTheta;
      matrix.m21 = 0;
      matrix.m22 = (float) cTheta;
   }

   private void createRandomRotationMatrixY(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = cTheta;
      matrix.m01 = 0;
      matrix.m02 = sTheta;
      matrix.m10 = 0;
      matrix.m11 = 1;
      matrix.m12 = 0;
      matrix.m20 = -sTheta;
      matrix.m21 = 0;
      matrix.m22 = cTheta;
   }

   private void createRandomRotationMatrixZ(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = cTheta;
      matrix.m01 = -sTheta;
      matrix.m02 = 0;
      matrix.m10 = sTheta;
      matrix.m11 = cTheta;
      matrix.m12 = 0;
      matrix.m20 = 0;
      matrix.m21 = 0;
      matrix.m22 = 1;
   }

   private void createRandomRotationMatrixZ(Random random, Matrix3f matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = (float) cTheta;
      matrix.m01 = (float) -sTheta;
      matrix.m02 = 0;
      matrix.m10 = (float) sTheta;
      matrix.m11 = (float) cTheta;
      matrix.m12 = 0;
      matrix.m20 = 0;
      matrix.m21 = 0;
      matrix.m22 = 1;
   }

   private void createRandomTransform4Vector(Vector4d vector, Random random)
   {
      vector.x = random.nextDouble();
      vector.y = random.nextDouble();
      vector.z = random.nextDouble();
      vector.w = 1;
   }

   private void createRandomTransform4Vector(Vector4f vector, Random random)
   {
      vector.x = random.nextFloat();
      vector.y = random.nextFloat();
      vector.z = random.nextFloat();
      vector.w = 1;
   }

   private float[] putDoublesInFloatArray(double[] matrixAsDoubleArray)
   {
      float[] ret = new float[matrixAsDoubleArray.length];
      for (int i = 0; i < matrixAsDoubleArray.length; i++)
      {
         ret[i] = (float) matrixAsDoubleArray[i];
      }

      return ret;
   }

   private void createFloatArrayFromMatrix4d(float[] floatArray,
         Matrix4f matrix)
   {
      floatArray[0] = matrix.m00;
      floatArray[1] = matrix.m01;
      floatArray[2] = matrix.m02;
      floatArray[3] = matrix.m03;
      floatArray[4] = matrix.m10;
      floatArray[5] = matrix.m11;
      floatArray[6] = matrix.m12;
      floatArray[7] = matrix.m13;
      floatArray[8] = matrix.m20;
      floatArray[9] = matrix.m21;
      floatArray[10] = matrix.m22;
      floatArray[11] = matrix.m23;
      floatArray[12] = matrix.m30;
      floatArray[13] = matrix.m31;
      floatArray[14] = matrix.m32;
      floatArray[15] = matrix.m33;
   }

   private void messWithRotationMatrixOrthogonality(Matrix4d matrix,
         Random random)
   {
      matrix.m00 += random.nextDouble() * 1e-3;
      matrix.m01 += random.nextDouble() * 1e-3;
      matrix.m02 += random.nextDouble() * 1e-3;
      matrix.m10 += random.nextDouble() * 1e-3;
      matrix.m11 += random.nextDouble() * 1e-3;
      matrix.m12 += random.nextDouble() * 1e-3;
      matrix.m20 += random.nextDouble() * 1e-3;
      matrix.m21 += random.nextDouble() * 1e-3;
      matrix.m22 += random.nextDouble() * 1e-3;
   }

	private double getRandomAngleEpsilonToPiMinusEpsilon(Random random, double epsilon)
   {
      return epsilon + (Math.PI - 2.0*epsilon) * random.nextDouble();
   }

   private double[] generateArrayOfTestAnglesMinus2PiToPiButExcludePlusMinusPi()
   {      
      return AngleTools.generateArrayOfTestAngles(nTests, 0.0, true, false);
   }

   @EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetErrorImpl()
   {
      double EPS = 0.00001;
      Matrix3d R1  = new Matrix3d();
      Matrix3d R2  = new Matrix3d();
      Matrix3d temp = new Matrix3d();
      Quat4d quat = new Quat4d();
      Vector3d err = new Vector3d();
      
      R1.setIdentity();
      R2.setIdentity();
        
      temp.rotY(0.4);
      err =  RigidBodyTransform.getRotationDifference(temp, R2);
      System.out.println( err );
      
      assertTrue(  err.epsilonEquals( new Vector3d(0, 0.4, 0),  EPS) );


      temp.rotZ(0.4);
      err = RigidBodyTransform.getRotationDifference(temp, R2);
      System.out.println( err );
      
      assertTrue(  err.epsilonEquals( new Vector3d(0, 0, 0.4),  EPS) );

      temp.rotX(0.4);
      err = RigidBodyTransform.getRotationDifference(temp, R2);
      System.out.println( err );   
      assertTrue(  err.epsilonEquals( new Vector3d( 0.4, 0, 0),  EPS) );
      
      temp.rotX(0.4);  R1.mul(temp);
      temp.rotY(0.8);  R1.mul(temp);     
      temp.rotZ(1.2);  R1.mul(temp);   
      
      System.out.println("..........");
      quat.set( R1 );
      System.out.println("R1: " + quat );
      
      //----------------
      temp.rotZ(0.3);  R2.mul(temp);     
      temp.rotY(0.6);  R2.mul(temp);   
      temp.rotX(0.9);  R2.mul(temp);
 
      quat.set( R2 );
      System.out.println("R2: " + quat );
      
      err = RigidBodyTransform.getRotationDifference(R1, R2);
      System.out.println("error: " + err );

      assertTrue(  err.epsilonEquals( new Vector3d( 0.5219536449894595, -0.546598672388116, 0.975076469006518 ),  EPS) );
      
      
      err = RigidBodyTransform.getRotationDifference(R1, R1);
      System.out.println("error: " + err );

      assertTrue("identity failed :" ,  err.epsilonEquals( new Vector3d( 0.0, 0.0, 0.0 ),  EPS) );
   }

   public static void assertTransformEquals(RigidBodyTransform expectedTransform, RigidBodyTransform actualTransform, double epsilon)
   {
      Matrix4d expected = new Matrix4d();
      Matrix4d actual = new Matrix4d();
      if (DEBUG)
         System.out.println("JUnitTools.assertTransformsEquals : expectedTransform = " + expectedTransform);
      if (DEBUG)
         System.out.println("JUnitTools.assertTransformsEquals : actualTransform = " + actualTransform);
      expectedTransform.get(expected);
      actualTransform.get(actual);
      if (DEBUG)
         System.out.println("JUnitTools.assertTransformsEquals : expected = " + expected);
      if (DEBUG)
         System.out.println("JUnitTools.assertTransformsEquals : actual = " + actual);

      try
      {
         assertEquals(expected.m00, actual.m00, epsilon);
         assertEquals(expected.m01, actual.m01, epsilon);
         assertEquals(expected.m02, actual.m02, epsilon);
         assertEquals(expected.m03, actual.m03, epsilon);
         assertEquals(expected.m10, actual.m10, epsilon);
         assertEquals(expected.m11, actual.m11, epsilon);
         assertEquals(expected.m12, actual.m12, epsilon);
         assertEquals(expected.m13, actual.m13, epsilon);
         assertEquals(expected.m20, actual.m20, epsilon);
         assertEquals(expected.m21, actual.m21, epsilon);
         assertEquals(expected.m22, actual.m22, epsilon);
         assertEquals(expected.m23, actual.m23, epsilon);
         assertEquals(expected.m30, actual.m30, epsilon);
         assertEquals(expected.m31, actual.m31, epsilon);
         assertEquals(expected.m32, actual.m32, epsilon);
         assertEquals(expected.m33, actual.m33, epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedTransform + ">\n but was:\n<" + actualTransform + ">");
      }
   }

}
