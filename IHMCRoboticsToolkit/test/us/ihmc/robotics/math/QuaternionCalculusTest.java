package us.ihmc.robotics.math;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class QuaternionCalculusTest
{
   private static final double EPSILON = 1.0e-10;

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConversionQDotToAngularVelocityBackAndForth() throws Exception
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quat4d q = RandomTools.generateRandomQuaternion(random);
         double length = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         Vector3d expectedAngularVelocity = RandomTools.generateRandomVector(random, length);
         if (random.nextBoolean())
            expectedAngularVelocity.negate();
         Vector3d actualAngularVelocity = new Vector3d();
         Quat4d qDot = new Quat4d();

         quaternionCalculus.computeQDot(q, expectedAngularVelocity, qDot);
         quaternionCalculus.computeAngularVelocity(q, qDot, actualAngularVelocity);

         assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, EPSILON));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConversionQDDotToAngularAccelerationBackAndForth() throws Exception
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quat4d q = RandomTools.generateRandomQuaternion(random);
         double length = RandomTools.generateRandomDouble(random, 0.0, 10.0);
         Vector3d angularVelocity = RandomTools.generateRandomVector(random, length);
         if (random.nextBoolean())
            angularVelocity.negate();
         Vector3d expectedAngularAcceleration = RandomTools.generateRandomVector(random, length);
         if (random.nextBoolean())
            expectedAngularAcceleration.negate();
         Vector3d actualAngularAcceleration = new Vector3d();
         Quat4d qDot = new Quat4d();
         Quat4d qDDot = new Quat4d();

         quaternionCalculus.computeQDot(q, angularVelocity, qDot);

         quaternionCalculus.computeQDDot(q, qDot, expectedAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, EPSILON));

         quaternionCalculus.computeQDDot(q, angularVelocity, actualAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, EPSILON));

         quaternionCalculus.computeQDDot(q, qDot, angularVelocity, actualAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, EPSILON));

         quaternionCalculus.computeQDDot(q, qDot, expectedAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDDot, angularVelocity, actualAngularAcceleration);
         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, EPSILON));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVelocityFromFDAgainstTrajectory() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      SimpleOrientationTrajectoryGenerator traj = new SimpleOrientationTrajectoryGenerator("traj", ReferenceFrame.getWorldFrame(), new YoVariableRegistry("null"));
      double trajectoryTime = 1.0;
      traj.setTrajectoryTime(trajectoryTime);
      Random random = new Random(65265L);
      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, ReferenceFrame.getWorldFrame());
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, ReferenceFrame.getWorldFrame());
      traj.setInitialOrientation(initialOrientation);
      traj.setFinalOrientation(finalOrientation);
      traj.initialize();

      double dt = 1.0e-4;
      double dtForFD = 1.0e-6;

      FrameOrientation orientation = new FrameOrientation();
      FrameVector expectedAngularVelocity = new FrameVector();
      Quat4d q = new Quat4d();
      Quat4d qDot = new Quat4d();
      Quat4d qPrevious = new Quat4d();
      Quat4d qNext = new Quat4d();
      Vector3d actualAngularVelocity = new Vector3d();

      for (double time = dt; time <= trajectoryTime - dt; time += dt)
      {
         traj.compute(time);
         traj.get(orientation);
         traj.packAngularVelocity(expectedAngularVelocity);
         orientation.getQuaternion(q);
         traj.compute(time - dtForFD);
         traj.get(orientation);
         orientation.getQuaternion(qPrevious);
         traj.compute(time + dtForFD);
         traj.get(orientation);
         orientation.getQuaternion(qNext);

         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dtForFD, qDot);
         quaternionCalculus.computeAngularVelocity(q, qDot, actualAngularVelocity);

         assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, 1.0e-8));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAccelerationFromFDAgainstTrajectory() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      SimpleOrientationTrajectoryGenerator traj = new SimpleOrientationTrajectoryGenerator("traj", ReferenceFrame.getWorldFrame(), new YoVariableRegistry("null"));
      double trajectoryTime = 1.0;
      traj.setTrajectoryTime(trajectoryTime);
      Random random = new Random(65265L);
      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, ReferenceFrame.getWorldFrame());
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, ReferenceFrame.getWorldFrame());
      traj.setInitialOrientation(initialOrientation);
      traj.setFinalOrientation(finalOrientation);
      traj.initialize();

      double dt = 1.0e-4;
      double dtForFD = 1.0e-4;

      FrameOrientation orientation = new FrameOrientation();
      FrameVector expectedAngularAcceleration = new FrameVector();
      Quat4d q = new Quat4d();
      Quat4d qDot = new Quat4d();
      Quat4d qDDot = new Quat4d();
      Quat4d qPrevious = new Quat4d();
      Quat4d qNext = new Quat4d();
      Vector3d actualAngularAcceleration = new Vector3d();

      for (double time = dt; time <= trajectoryTime - dt; time += dt)
      {
         traj.compute(time);
         traj.get(orientation);
         traj.packAngularAcceleration(expectedAngularAcceleration);
         orientation.getQuaternion(q);
         traj.compute(time - dtForFD);
         traj.get(orientation);
         orientation.getQuaternion(qPrevious);
         traj.compute(time + dtForFD);
         traj.get(orientation);
         orientation.getQuaternion(qNext);

         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dtForFD, qDot);
         quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qPrevious, q, qNext, dtForFD, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);

         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, 1.0e-5));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInterpolateAgainstQuat4d() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Quat4d q0 = RandomTools.generateRandomQuaternion(random);
      Quat4d q1 = RandomTools.generateRandomQuaternion(random);
      Quat4d expectedQInterpolated = new Quat4d();
      Quat4d actualQInterpolated = new Quat4d();

      for (double alpha = 0.0; alpha <= 1.0; alpha += 1.0e-6)
      {
         expectedQInterpolated.interpolate(q0, q1, alpha);
         quaternionCalculus.interpolate(alpha, q0, q1, actualQInterpolated);

         assertTrue(expectedQInterpolated.epsilonEquals(actualQInterpolated, EPSILON));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTransformOnVector() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Vector3d expectedTransformedVector = new Vector3d();
      Vector3d actualTransformedVector = new Vector3d();

      for (int i = 0; i < 1000; i++)
      {
         Quat4d qForTransform = RandomTools.generateRandomQuaternion(random);
         qForTransform.normalize();
         Matrix3d matrixForTranform = new Matrix3d();
         matrixForTranform.set(qForTransform);
         Vector3d originalVector = RandomTools.generateRandomVector(random, new Vector3d(-10.0, -10.0, -10.0), new Vector3d(10.0, 10.0, 10.0));
         if (random.nextBoolean())
            originalVector.negate();
         matrixForTranform.transform(originalVector, expectedTransformedVector);
         quaternionCalculus.transform(qForTransform, originalVector, actualTransformedVector);
         assertEquals(expectedTransformedVector.length(), actualTransformedVector.length(), EPSILON);
         assertTrue(expectedTransformedVector.epsilonEquals(actualTransformedVector, EPSILON));

         actualTransformedVector.set(originalVector);
         quaternionCalculus.transform(qForTransform, actualTransformedVector);
         assertEquals(expectedTransformedVector.length(), actualTransformedVector.length(), EPSILON);
         assertTrue(expectedTransformedVector.epsilonEquals(actualTransformedVector, EPSILON));
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInvertTransformOnVector() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Vector3d expectedTransformedVector = new Vector3d();
      Vector3d actualTransformedVector = new Vector3d();

      for (int i = 0; i < 1000; i++)
      {
         Quat4d qForTransform = RandomTools.generateRandomQuaternion(random);
         qForTransform.normalize();
         Matrix3d matrixForTranform = new Matrix3d();
         matrixForTranform.set(qForTransform);
         matrixForTranform.transpose();
         Vector3d originalVector = RandomTools.generateRandomVector(random, new Vector3d(-10.0, -10.0, -10.0), new Vector3d(10.0, 10.0, 10.0));
         if (random.nextBoolean())
            originalVector.negate();
         matrixForTranform.transform(originalVector, expectedTransformedVector);
         quaternionCalculus.invertTransform(qForTransform, originalVector, actualTransformedVector);
         assertEquals(expectedTransformedVector.length(), actualTransformedVector.length(), EPSILON);
         assertTrue(expectedTransformedVector.epsilonEquals(actualTransformedVector, EPSILON));

         actualTransformedVector.set(originalVector);
         quaternionCalculus.invertTransform(qForTransform, actualTransformedVector);
         assertEquals(expectedTransformedVector.length(), actualTransformedVector.length(), EPSILON);
         assertTrue(expectedTransformedVector.epsilonEquals(actualTransformedVector, EPSILON));
      }
   }

   @Test
   public void testTransformOnQuaternion() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Quat4d actualTransformedQuaternion = new Quat4d();

      Matrix3d originalMatrix = new Matrix3d();
      Matrix3d expectedTransformedMatrix = new Matrix3d();
      Matrix3d actualTransformedMatrix = new Matrix3d();

      for (int i = 0; i < 1000; i++)
      {
         Quat4d qForTransform = RandomTools.generateRandomQuaternion(random);
         qForTransform.normalize();
         Matrix3d matrixForTranform = new Matrix3d();
         matrixForTranform.set(qForTransform);
         Quat4d originalQuat = RandomTools.generateRandomQuaternion(random);
         if (random.nextBoolean())
            originalQuat.negate();
         originalMatrix.set(originalQuat);
         expectedTransformedMatrix.mul(matrixForTranform, originalMatrix);
         expectedTransformedMatrix.mulTransposeRight(expectedTransformedMatrix, matrixForTranform);
         quaternionCalculus.transform(qForTransform, originalQuat, actualTransformedQuaternion);
         actualTransformedMatrix.set(actualTransformedQuaternion);
         assertTrue(expectedTransformedMatrix.epsilonEquals(actualTransformedMatrix, EPSILON));

         actualTransformedQuaternion.set(originalQuat);
         quaternionCalculus.transform(qForTransform, actualTransformedQuaternion);
         actualTransformedMatrix.set(actualTransformedQuaternion);
         assertTrue(expectedTransformedMatrix.epsilonEquals(actualTransformedMatrix, EPSILON));
      }
   }

   @Test
   public void testInvertTransformOnQuaternion() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Quat4d actualTransformedQuaternion = new Quat4d();

      Matrix3d originalMatrix = new Matrix3d();
      Matrix3d expectedTransformedMatrix = new Matrix3d();
      Matrix3d actualTransformedMatrix = new Matrix3d();

      for (int i = 0; i < 1000; i++)
      {
         Quat4d qForTransform = RandomTools.generateRandomQuaternion(random);
         qForTransform.normalize();
         Matrix3d matrixForTranform = new Matrix3d();
         matrixForTranform.set(qForTransform);
         matrixForTranform.transpose();
         Quat4d originalQuat = RandomTools.generateRandomQuaternion(random);
         if (random.nextBoolean())
            originalQuat.negate();
         originalMatrix.set(originalQuat);
         expectedTransformedMatrix.mul(matrixForTranform, originalMatrix);
         expectedTransformedMatrix.mulTransposeRight(expectedTransformedMatrix, matrixForTranform);
         quaternionCalculus.invertTransform(qForTransform, originalQuat, actualTransformedQuaternion);
         actualTransformedMatrix.set(actualTransformedQuaternion);
         assertTrue(expectedTransformedMatrix.epsilonEquals(actualTransformedMatrix, EPSILON));

         actualTransformedQuaternion.set(originalQuat);
         quaternionCalculus.invertTransform(qForTransform, actualTransformedQuaternion);
         actualTransformedMatrix.set(actualTransformedQuaternion);
         assertTrue(expectedTransformedMatrix.epsilonEquals(actualTransformedMatrix, EPSILON));
      }
   }

}
