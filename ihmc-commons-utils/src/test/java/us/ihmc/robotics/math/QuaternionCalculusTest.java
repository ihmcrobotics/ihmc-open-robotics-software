package us.ihmc.robotics.math;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import static java.lang.Math.PI;
import static org.junit.jupiter.api.Assertions.*;

public class QuaternionCalculusTest
{
   private static final double EPSILON = 1.0e-10;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testLogAndExpAlgebra()
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);

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
   

   @Test
   public void testConversionQDotToAngularVelocityBackAndForth()
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         double length = RandomNumbers.nextDouble(random, 0.0, 10.0);
         Vector3D expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random, length);
         if (random.nextBoolean())
            expectedAngularVelocity.negate();
         Vector3D actualAngularVelocity = new Vector3D();
         Vector4D qDot = new Vector4D();

         quaternionCalculus.computeQDotInWorldFrame(q, expectedAngularVelocity, qDot);
         quaternionCalculus.computeAngularVelocityInWorldFrame(q, qDot, actualAngularVelocity);

         EuclidCoreTestTools.assertEquals(expectedAngularVelocity, actualAngularVelocity, EPSILON);
      }
   }

   @Test
   public void testConversionQDDotToAngularAccelerationBackAndForth()
   {
      Random random = new Random(651651961L);

      for (int i = 0; i < 10000; i++)
      {
         QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         double length = RandomNumbers.nextDouble(random, 0.0, 10.0);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random, length);
         if (random.nextBoolean())
            angularVelocity.negate();
         Vector3D expectedAngularAcceleration = EuclidCoreRandomTools.nextVector3D(random, length);
         if (random.nextBoolean())
            expectedAngularAcceleration.negate();
         Vector3D actualAngularAcceleration = new Vector3D();
         Vector4D qDot = new Vector4D();
         Vector4D qDDot = new Vector4D();

         quaternionCalculus.computeQDotInWorldFrame(q, angularVelocity, qDot);

         quaternionCalculus.computeQDDotInWorldFrame(q, qDot, expectedAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
         EuclidCoreTestTools.assertEquals(expectedAngularAcceleration, actualAngularAcceleration, EPSILON);

         quaternionCalculus.computeQDDotInWorldFrame(q, angularVelocity, actualAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
         EuclidCoreTestTools.assertEquals(expectedAngularAcceleration, actualAngularAcceleration, EPSILON);

         quaternionCalculus.computeQDDotInWorldFrame(q, qDot, angularVelocity, actualAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
         EuclidCoreTestTools.assertEquals(expectedAngularAcceleration, actualAngularAcceleration, EPSILON);

         quaternionCalculus.computeQDDotInWorldFrame(q, qDot, expectedAngularAcceleration, qDDot);
         quaternionCalculus.computeAngularAccelerationInWorldFrame(q, qDDot, angularVelocity, actualAngularAcceleration);
         EuclidCoreTestTools.assertEquals(expectedAngularAcceleration, actualAngularAcceleration, EPSILON);
      }
   }

//   @Test
//   public void testVelocityFromFDAgainstTrajectory()
//   {
//      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
//      SimpleOrientationTrajectoryGenerator traj = new SimpleOrientationTrajectoryGenerator("traj", ReferenceFrame.getWorldFrame(), new YoRegistry("null"));
//      double trajectoryTime = 1.0;
//      traj.setTrajectoryTime(trajectoryTime);
//      Random random = new Random(65265L);
//      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
//      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
//      traj.setInitialOrientation(initialOrientation);
//      traj.setFinalOrientation(finalOrientation);
//      traj.initialize();
//
//      double dt = 1.0e-4;
//      double dtForFD = 1.0e-6;
//
//      FrameQuaternion orientation = new FrameQuaternion();
//      FrameVector3D expectedAngularVelocity = new FrameVector3D();
//      Quaternion q = new Quaternion();
//      Vector4D qDot = new Vector4D();
//      Quaternion qPrevious = new Quaternion();
//      Quaternion qNext = new Quaternion();
//      Vector3D actualAngularVelocity = new Vector3D();
//
//      for (double time = dt; time <= trajectoryTime - dt; time += dt)
//      {
//         traj.compute(time);
//         expectedAngularVelocity.setIncludingFrame(traj.getAngularVelocity());
//         q.set(traj.getOrientation());
//         traj.compute(time - dtForFD);
//         qPrevious.set(traj.getOrientation());
//         traj.compute(time + dtForFD);
//         qNext.set(traj.getOrientation());
//
//         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dtForFD, qDot);
//         quaternionCalculus.computeAngularVelocityInWorldFrame(q, qDot, actualAngularVelocity);
//
//         EuclidCoreTestTools.assertEquals(expectedAngularVelocity, actualAngularVelocity, 1e-8);
//      }
//   }

   @Test
   public void testFDSimpleCase() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(65265L);
      double integrationTime = 1.0;
      double angleVelocity = RandomNumbers.nextDouble(random, 0.0, 2.0 * PI) / integrationTime;
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
         axisAnglePrevious.setAngle(trimAngleMinusPiToPi(angleVelocity * (time - dt)) - PI);
         qPrevious.set(axisAnglePrevious);
         axisAngleCurrent.setAngle(trimAngleMinusPiToPi(angleVelocity * time) - PI);
         qCurrent.set(axisAngleCurrent);
         axisAngleNext.setAngle(trimAngleMinusPiToPi(angleVelocity * (time + dt)) - PI);
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
         EuclidCoreTestTools.assertEquals(expectedAngularAcceleration, actualAngularAcceleration, 1e-7);
      }
   }

//   @Test
//   public void testAccelerationFromFDAgainstTrajectory()
//   {
//      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
//      SimpleOrientationTrajectoryGenerator traj = new SimpleOrientationTrajectoryGenerator("traj", ReferenceFrame.getWorldFrame(), new YoRegistry("null"));
//      double trajectoryTime = 1.0;
//      traj.setTrajectoryTime(trajectoryTime);
//      Random random = new Random(65265L);
//      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
//      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
//      traj.setInitialOrientation(initialOrientation);
//      traj.setFinalOrientation(finalOrientation);
//      traj.initialize();
//
//      double dt = 1.0e-4;
//      double dtForFD = 1.0e-4;
//
//      FrameQuaternion orientation = new FrameQuaternion();
//      FrameVector3D expectedAngularAcceleration = new FrameVector3D();
//      Quaternion q = new Quaternion();
//      Vector4D qDot = new Vector4D();
//      Vector4D qDDot = new Vector4D();
//      Quaternion qPrevious = new Quaternion();
//      Quaternion qNext = new Quaternion();
//      Vector3D actualAngularAcceleration = new Vector3D();
//
//      for (double time = dt; time <= trajectoryTime - dt; time += dt)
//      {
//         traj.compute(time);
//         expectedAngularAcceleration.setIncludingFrame(traj.getAngularAcceleration());
//         q.set(traj.getOrientation());
//         traj.compute(time - dtForFD);
//         qPrevious.set(traj.getOrientation());
//         traj.compute(time + dtForFD);
//         qNext.set(traj.getOrientation());
//
//         quaternionCalculus.computeQDotByFiniteDifferenceCentral(qPrevious, qNext, dtForFD, qDot);
//         quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qPrevious, q, qNext, dtForFD, qDDot);
//         quaternionCalculus.computeAngularAcceleration(q, qDot, qDDot, actualAngularAcceleration);
//
//         EuclidCoreTestTools.assertEquals(expectedAngularAcceleration, actualAngularAcceleration, 1e-5);
//      }
//   }

   @Test
   public void testInterpolateAgainstQuat4d()
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Random random = new Random(6546545L);
      Quaternion q0 = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion q1 = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion expectedQInterpolated = new Quaternion();
      Quaternion actualQInterpolated = new Quaternion();

      for (double alpha = 0.0; alpha <= 1.0; alpha += 1.0e-6)
      {
         expectedQInterpolated.interpolate(q0, q1, alpha);
         quaternionCalculus.interpolate(alpha, q0, q1, actualQInterpolated);

         EuclidCoreTestTools.assertEquals(expectedQInterpolated, actualQInterpolated, EPSILON);
      }
   }


   public static double trimAngleMinusPiToPi(double angle)
   {
      return shiftAngleToStartOfRange(angle, -PI);
   }

   static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange)
   {
      return shiftAngleToStartOfRange(angleToShift, startOfAngleRange, 2.0 * Math.PI);
   }

   static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange, double endOfAngleRange)
   {
      double ret = angleToShift;
      startOfAngleRange = startOfAngleRange - EPSILON;

      if (angleToShift < startOfAngleRange)
      {
         ret = angleToShift + Math.ceil((startOfAngleRange - angleToShift) / endOfAngleRange) * endOfAngleRange;
      }

      if (angleToShift >= (startOfAngleRange + endOfAngleRange))
      {
         ret = angleToShift - Math.floor((angleToShift - startOfAngleRange) / endOfAngleRange) * endOfAngleRange;
      }

      return ret;
   }
   @Test
   public void testShiftAngleToStartOfRangeUnitless()
   {
      double range = Math.pow(2.0, 13.0);

      double endOfAngleRange = range;
      double angleToShift = 1.6 * range;
      double expectedReturn = 0.6 * range;
      double actualReturn = shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      angleToShift = -0.4 * range;
      expectedReturn = 0.6 * range;
      actualReturn = shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      angleToShift = 0.4 * range;
      expectedReturn = 0.4 * range;
      actualReturn = shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      int iters = 1000;
      Random random = new Random();
      for (int i = 0; i < iters; i++)
      {
         double ratio = -6.0 + 12.0 * random.nextDouble();

         angleToShift = ratio * range;
         expectedReturn = angleToShift;

         if (angleToShift < 0.0)
            expectedReturn = angleToShift + Math.ceil((-angleToShift) / endOfAngleRange) * endOfAngleRange;

         if (angleToShift >= endOfAngleRange)
            expectedReturn = angleToShift - Math.floor((angleToShift) / endOfAngleRange) * endOfAngleRange;

         actualReturn = shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
         assertEquals(expectedReturn, actualReturn, 1e-12);
      }
   }

}
