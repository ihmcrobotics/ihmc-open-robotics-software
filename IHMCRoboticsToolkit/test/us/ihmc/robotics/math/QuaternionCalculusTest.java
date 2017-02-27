package us.ihmc.robotics.math;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class QuaternionCalculusTest
{
   private static final double EPSILON = 1.0e-10;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogAndExpAlgebra() throws Exception
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quaternion q = RandomTools.generateRandomQuaternion(random);

         Vector4D qLog = new Vector4D();
         Quaternion vExp = new Quaternion();
         
         quaternionCalculus.log(q, qLog);
         Vector3D v = new Vector3D(qLog.getX(),qLog.getY(),qLog.getZ()); 
         
         quaternionCalculus.exp(v, vExp);

         assertTrue(Math.abs(q.getX() - vExp.getX()) < 10e-10);
         assertTrue(Math.abs(q.getY() - vExp.getY()) < 10e-10);
         assertTrue(Math.abs(q.getZ() - vExp.getZ()) < 10e-10);
         assertTrue(Math.abs(q.getS() - vExp.getS()) < 10e-10);

      }
   }
   

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConversionQDotToAngularVelocityBackAndForth() throws Exception
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quaternion q = RandomTools.generateRandomQuaternion(random);
         double length = RandomNumbers.nextDouble(random, 0.0, 10.0);
         Vector3D expectedAngularVelocity = RandomTools.generateRandomVector(random, length);
         if (random.nextBoolean())
            expectedAngularVelocity.negate();
         Vector3D actualAngularVelocity = new Vector3D();
         Vector4D qDot = new Vector4D();

         quaternionCalculus.computeQDot(q, expectedAngularVelocity, qDot);
         quaternionCalculus.computeAngularVelocityInWorldFrame(q, qDot, actualAngularVelocity);

         assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, EPSILON));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConversionQDDotToAngularAccelerationBackAndForth() throws Exception
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quaternion q = RandomTools.generateRandomQuaternion(random);
         double length = RandomNumbers.nextDouble(random, 0.0, 10.0);
         Vector3D angularVelocity = RandomTools.generateRandomVector(random, length);
         if (random.nextBoolean())
            angularVelocity.negate();
         Vector3D expectedAngularAcceleration = RandomTools.generateRandomVector(random, length);
         if (random.nextBoolean())
            expectedAngularAcceleration.negate();
         Vector3D actualAngularAcceleration = new Vector3D();
         Vector4D qDot = new Vector4D();
         Vector4D qDDot = new Vector4D();

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

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
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
      Quaternion q = new Quaternion();
      Vector4D qDot = new Vector4D();
      Quaternion qPrevious = new Quaternion();
      Quaternion qNext = new Quaternion();
      Vector3D actualAngularVelocity = new Vector3D();

      for (double time = dt; time <= trajectoryTime - dt; time += dt)
      {
         traj.compute(time);
         traj.getOrientation(orientation);
         traj.getAngularVelocity(expectedAngularVelocity);
         orientation.getQuaternion(q);
         traj.compute(time - dtForFD);
         traj.getOrientation(orientation);
         orientation.getQuaternion(qPrevious);
         traj.compute(time + dtForFD);
         traj.getOrientation(orientation);
         orientation.getQuaternion(qNext);

         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dtForFD, qDot);
         quaternionCalculus.computeAngularVelocityInWorldFrame(q, qDot, actualAngularVelocity);

         assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, 1.0e-8));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFDSimpleCase() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(65265L);
      double integrationTime = 1.0;
      double angleVelocity = RandomNumbers.nextDouble(random, 0.0, 2.0 * Math.PI) / integrationTime;
      Vector3D expectedAngularVelocity = new Vector3D(angleVelocity, 0.0, 0.0);
      Vector3D expectedAngularAcceleration = new Vector3D();
      AxisAngle axisAnglePrevious = new AxisAngle(1.0, 0.0, 0.0, 0.0);
      AxisAngle axisAngleCurrent = new AxisAngle(1.0, 0.0, 0.0, 0.0);
      AxisAngle axisAngleNext = new AxisAngle(1.0, 0.0, 0.0, 0.0);
      Quaternion qPrevious = new Quaternion();
      Quaternion qCurrent = new Quaternion();
      Quaternion qNext = new Quaternion();
      Vector4D qDot = new Vector4D();
      Vector4D qDDot = new Vector4D();

      Vector3D actualAngularVelocity = new Vector3D();
      Vector3D actualAngularAcceleration = new Vector3D();

      double dt = 1.0e-4;
      for (double time = dt; time < integrationTime; time += dt)
      {
         axisAnglePrevious.setAngle(AngleTools.trimAngleMinusPiToPi(angleVelocity * (time - dt)) - Math.PI);
         qPrevious.set(axisAnglePrevious);
         axisAngleCurrent.setAngle(AngleTools.trimAngleMinusPiToPi(angleVelocity * time) - Math.PI);
         qCurrent.set(axisAngleCurrent);
         axisAngleNext.setAngle(AngleTools.trimAngleMinusPiToPi(angleVelocity * (time + dt)) - Math.PI);
         qNext.set(axisAngleNext);

         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dt, qDot);
         quaternionCalculus.computeAngularVelocityInWorldFrame(qCurrent, qDot, actualAngularVelocity);

         quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qPrevious, qCurrent, qNext, dt, qDDot);
         quaternionCalculus.computeAngularAcceleration(qCurrent, qDot, qDDot, actualAngularAcceleration);

         boolean sameVelocity = expectedAngularVelocity.epsilonEquals(actualAngularVelocity, 1.0e-7);
         if (!sameVelocity)
         {
            System.out.println("Expected angular velocity: " + expectedAngularVelocity);
            System.out.println("Actual   angular velocity: " + actualAngularVelocity);
         }
         assertTrue(sameVelocity);
         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, 1.0e-7));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
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
      Quaternion q = new Quaternion();
      Vector4D qDot = new Vector4D();
      Vector4D qDDot = new Vector4D();
      Quaternion qPrevious = new Quaternion();
      Quaternion qNext = new Quaternion();
      Vector3D actualAngularAcceleration = new Vector3D();

      for (double time = dt; time <= trajectoryTime - dt; time += dt)
      {
         traj.compute(time);
         traj.getOrientation(orientation);
         traj.getAngularAcceleration(expectedAngularAcceleration);
         orientation.getQuaternion(q);
         traj.compute(time - dtForFD);
         traj.getOrientation(orientation);
         orientation.getQuaternion(qPrevious);
         traj.compute(time + dtForFD);
         traj.getOrientation(orientation);
         orientation.getQuaternion(qNext);

         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dtForFD, qDot);
         quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qPrevious, q, qNext, dtForFD, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);

         assertTrue(expectedAngularAcceleration.epsilonEquals(actualAngularAcceleration, 1.0e-5));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testInterpolateAgainstQuat4d() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Quaternion q0 = RandomTools.generateRandomQuaternion(random);
      Quaternion q1 = RandomTools.generateRandomQuaternion(random);
      Quaternion expectedQInterpolated = new Quaternion();
      Quaternion actualQInterpolated = new Quaternion();

      for (double alpha = 0.0; alpha <= 1.0; alpha += 1.0e-6)
      {
         expectedQInterpolated.interpolate(q0, q1, alpha);
         quaternionCalculus.interpolate(alpha, q0, q1, actualQInterpolated);

         assertTrue(expectedQInterpolated.epsilonEquals(actualQInterpolated, EPSILON));
      }
   }
}
