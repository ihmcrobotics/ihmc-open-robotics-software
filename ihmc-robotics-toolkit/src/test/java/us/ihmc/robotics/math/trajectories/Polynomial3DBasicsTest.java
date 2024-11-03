package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Assertions;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;

import java.awt.*;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public abstract class Polynomial3DBasicsTest
{
   protected static final int ITERATIONS = 1000;
   protected static final double SMALL_EPSILON = 1.0e-12;

   public abstract Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients);

   @Test
   public void testLinearSet()
   {
      Polynomial3DBasics traj = getPolynomial(2);
      assertEquals(2, traj.getMaximumNumberOfCoefficients());

      assertEquals(0, traj.getNumberOfCoefficients());
      Point3D start = new Point3D(3.0, 4.0, 5.0);
      Point3D end = new Point3D(6.0, 7.0, 8.0);
      traj.setLinear(1, 2, start, end);

      assertEquals(1, traj.getTimeInterval().getStartTime(), SMALL_EPSILON);
      assertEquals(2, traj.getTimeInterval().getEndTime(), SMALL_EPSILON);
      traj.compute(traj.getTimeInterval().getStartTime());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(start, traj.getPosition(), SMALL_EPSILON);
      traj.compute(traj.getTimeInterval().getEndTime());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(end, traj.getPosition(), SMALL_EPSILON);

//      assertEquals(2, traj.getCoefficient(1), SMALL_EPSILON);
//      assertEquals(1, traj.getCoefficient(0), SMALL_EPSILON);
   }

   @Test
   public void testTimeInterval()
   {
      Polynomial3DBasics traj = getPolynomial(2);
      assertEquals(2, traj.getMaximumNumberOfCoefficients());

      assertEquals(0, traj.getNumberOfCoefficients());
      Point3D start = new Point3D(3.0, 4.0, 5.0);
      Point3D end = new Point3D(6.0, 7.0, 8.0);
      traj.setLinear(1, 2, start, end);

      assertEquals(1, traj.getTimeInterval().getStartTime(), SMALL_EPSILON);
      assertEquals(2, traj.getTimeInterval().getEndTime(), SMALL_EPSILON);
      traj.compute(traj.getTimeInterval().getStartTime());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(start, traj.getPosition(), SMALL_EPSILON);
      traj.compute(traj.getTimeInterval().getEndTime());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(end, traj.getPosition(), SMALL_EPSILON);

      assertEquals(1.0, traj.getAxis(Axis3D.X).getTimeInterval().getStartTime(), SMALL_EPSILON);
      assertEquals(1.0, traj.getAxis(Axis3D.Y).getTimeInterval().getStartTime(), SMALL_EPSILON);
      assertEquals(1.0, traj.getAxis(Axis3D.Z).getTimeInterval().getStartTime(), SMALL_EPSILON);

      assertEquals(2.0, traj.getAxis(Axis3D.X).getTimeInterval().getEndTime(), SMALL_EPSILON);
      assertEquals(2.0, traj.getAxis(Axis3D.Y).getTimeInterval().getEndTime(), SMALL_EPSILON);
      assertEquals(2.0, traj.getAxis(Axis3D.Z).getTimeInterval().getEndTime(), SMALL_EPSILON);

      traj.getTimeInterval().setInterval(3.5, 7.2);

      assertEquals(3.5, traj.getAxis(Axis3D.X).getTimeInterval().getStartTime(), SMALL_EPSILON);
      assertEquals(3.5, traj.getAxis(Axis3D.Y).getTimeInterval().getStartTime(), SMALL_EPSILON);
      assertEquals(3.5, traj.getAxis(Axis3D.Z).getTimeInterval().getStartTime(), SMALL_EPSILON);

      assertEquals(7.2, traj.getAxis(Axis3D.X).getTimeInterval().getEndTime(), SMALL_EPSILON);
      assertEquals(7.2, traj.getAxis(Axis3D.Y).getTimeInterval().getEndTime(), SMALL_EPSILON);
      assertEquals(7.2, traj.getAxis(Axis3D.Z).getTimeInterval().getEndTime(), SMALL_EPSILON);

      //      assertEquals(2, traj.getCoefficient(1), SMALL_EPSILON);
      //      assertEquals(1, traj.getCoefficient(0), SMALL_EPSILON);
   }

   @Test
   public void testExceptionThrownAfterShift()
   {
      Polynomial3DBasics trajectory = getPolynomial(5);
      Point3D start = new Point3D(3.0, 4.0, 5.0);
      Point3D end = new Point3D(6.0, 7.0, 8.0);
      trajectory.setCubic(0.0, 5.0, start, new Vector3D(), end, new Vector3D());

      boolean caughtException = false;
      try
      {
         trajectory.initialize();
      }
      catch (RuntimeException e)
      {
         caughtException = true;
      }

      assertFalse(caughtException);

      trajectory.shiftTrajectory(0.5, 0.6, 0.76);

      try
      {
         trajectory.initialize();
      }
      catch (RuntimeException e)
      {
         caughtException = true;
      }

      assertTrue(caughtException);
   }

   @Test
   public void testSetConstant()
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 1, 10);
         Polynomial3DBasics trajectory = getPolynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + random.nextDouble();
         Point3DReadOnly z = EuclidCoreRandomTools.nextPoint3D(random);

         assertEquals(maxNumberOfCoefficients, trajectory.getMaximumNumberOfCoefficients());

         if (maxNumberOfCoefficients < 1)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setConstant(t0, tf, z));
            continue;
         }

         trajectory.setConstant(t0, tf, z);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(z, trajectory.getPosition(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetLinear() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 1, 10);
         Polynomial3DBasics trajectory = getPolynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + random.nextDouble();
         Point3DReadOnly z0 = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Point3DReadOnly zf = EuclidCoreRandomTools.nextPoint3D(random, 1.0);

         assertEquals(maxNumberOfCoefficients, trajectory.getMaximumNumberOfCoefficients());


         if (maxNumberOfCoefficients < 2)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setLinear(t0, tf, z0, zf));
            continue;
         }

         trajectory.setLinear(t0, tf, z0, zf);

         Vector3D zDot = new Vector3D();
         zDot.sub(zf, z0);
         zDot.scale(1.0 / (tf - t0));
         Polynomial3DBasics derivative = getPolynomial(1);
         derivative.setConstant(t0, tf, zDot);

         trajectory.compute(t0);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(z0, trajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);

         trajectory.compute(tf);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(zf, trajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            Point3D expected = new Point3D();
            expected.interpolate(z0, zf, (t - t0) / (tf - t0));
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expected, trajectory.getPosition(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);

            derivative.compute(t);
            EuclidCoreTestTools.assertEquals(derivative.getPosition(), trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);
         }

         trajectory.setLinear(t0, z0, zDot);

         derivative = getPolynomial(1);
         derivative.setConstant(t0, tf, zDot);

         trajectory.compute(t0);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(z0, trajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);

         trajectory.compute(tf);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(zf, trajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            Point3D expected = new Point3D();
            expected.interpolate(z0, zf, (t - t0) / (tf - t0));
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expected, trajectory.getPosition(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), trajectory.getAcceleration(), SMALL_EPSILON);

            derivative.compute(t);
            EuclidCoreTestTools.assertEquals(derivative.getPosition(), trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);
         }
      }
   }


   @Test
   public void testSetQuadratic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 1, 10);
         Polynomial3DBasics trajectory = getPolynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;
         Point3D z0 = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zd0 = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         Point3D zf = EuclidCoreRandomTools.nextPoint3D(random, 1.0);

         if (maxNumberOfCoefficients < 3)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setQuadratic(t0, tf, z0, zd0, zf));
            continue;
         }

         assertEquals(maxNumberOfCoefficients, trajectory.getMaximumNumberOfCoefficients());


         trajectory.setQuadratic(t0, tf, z0, zd0, zf);

         trajectory.compute(t0);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(z0, trajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zd0, trajectory.getVelocity(), SMALL_EPSILON);

         trajectory.compute(tf);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(zf, trajectory.getPosition(), SMALL_EPSILON);

         Polynomial3DBasics derivative = getPolynomial(2);
         derivative.setLinear(t0, tf, new Point3D(zd0), new Point3D(trajectory.getVelocity()));

         double dt = 1.0e-8;

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            derivative.compute(t);

            EuclidCoreTestTools.assertEquals(derivative.getPosition(), trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);

            trajectory.compute(t + dt);
            Point3DReadOnly nextPosition = new Point3D(trajectory.getPosition());
            trajectory.compute(t - dt);
            Point3DReadOnly lastPosition = new Point3D(trajectory.getPosition());
            Vector3D expectedVelocity = new Vector3D();
            expectedVelocity.sub(nextPosition, lastPosition);
            expectedVelocity.scale(0.5 / dt);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(expectedVelocity, trajectory.getVelocity(), 1.0e-6);

         }
      }
   }

   @Test
   public void testSetCubic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 1, 10);
         Polynomial3DBasics trajectory = getPolynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;

         Point3D z0 = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zd0 = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         Point3D zf = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zdf = EuclidCoreRandomTools.nextVector3D(random, 1.0);

         if (maxNumberOfCoefficients < 4)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setCubic(t0, tf, z0, zd0, zf, zdf));
            continue;
         }

         assertEquals(maxNumberOfCoefficients, trajectory.getMaximumNumberOfCoefficients());


         trajectory.setCubic(t0, tf, z0, zd0, zf, zdf);

         trajectory.compute(t0);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(z0, trajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zd0, trajectory.getVelocity(), SMALL_EPSILON);

         Polynomial3DBasics derivative = getPolynomial(3);
         derivative.setQuadratic(t0, tf, new Point3D(zd0), trajectory.getAcceleration(), new Point3D(zdf));

         trajectory.compute(tf);
         EuclidCoreTestTools.assertEquals(zf, trajectory.getPosition(), SMALL_EPSILON);

         double dt = 1.0e-8;

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            derivative.compute(t);

            EuclidCoreTestTools.assertEquals(derivative.getPosition(), trajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);

            trajectory.compute(t + dt);
            Point3DReadOnly nextPosition = new Point3D(trajectory.getPosition());
            trajectory.compute(t - dt);
            Point3DReadOnly lastPosition = new Point3D(trajectory.getPosition());
            Vector3D velocityExpected = new Vector3D();
            velocityExpected.sub(nextPosition, lastPosition);
            velocityExpected.scale(0.5 / dt);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocityExpected, trajectory.getVelocity(), 5.0e-6);

         }
      }
   }

   @Test
   public void testTransformCubic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 4, 10);
         Polynomial3DBasics transformedTrajectory = getPolynomial(maxNumberOfCoefficients);
         Polynomial3DBasics trajectoryWithTransformedInputs = getPolynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;


         Point3D z0 = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zd0 = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         Point3D zf = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zdf = EuclidCoreRandomTools.nextVector3D(random, 1.0);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         transformedTrajectory.setCubic(t0, tf, z0, zd0, zf, zdf);
         transformedTrajectory.applyTransform(transform);

         Point3D transformedz0 = new Point3D(z0);
         Vector3D transformedzd0 = new Vector3D(zd0);
         Point3D transformedzf = new Point3D(zf);
         Vector3D transformedzdf = new Vector3D(zdf);

         transformedz0.applyTransform(transform);
         transformedzd0.applyTransform(transform);
         transformedzf.applyTransform(transform);
         transformedzdf.applyTransform(transform);

         transformedTrajectory.compute(t0);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(transformedz0, transformedTrajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(transformedzd0, transformedTrajectory.getVelocity(), SMALL_EPSILON);

         transformedTrajectory.compute(tf);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(transformedzf, transformedTrajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(transformedzdf, transformedTrajectory.getVelocity(), SMALL_EPSILON);

         trajectoryWithTransformedInputs.setCubic(t0, tf, transformedz0, transformedzd0, transformedzf, transformedzdf);

         double dt = 1.0e-3;

         for (double t = t0; t <= tf; t += dt)
         {
            transformedTrajectory.compute(t);
            trajectoryWithTransformedInputs.compute(t);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectoryWithTransformedInputs.getPosition(), transformedTrajectory.getPosition(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectoryWithTransformedInputs.getVelocity(), transformedTrajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectoryWithTransformedInputs.getAcceleration(), transformedTrajectory.getAcceleration(), SMALL_EPSILON);

         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 4, 10);
         Polynomial3DBasics transformedTrajectory = getPolynomial(maxNumberOfCoefficients);
         Polynomial3DBasics trajectoryWithTransformedInputs = getPolynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;


         Point3D z0 = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zd0 = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         Point3D zf = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
         Vector3D zdf = EuclidCoreRandomTools.nextVector3D(random, 1.0);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         transformedTrajectory.setCubic(t0, tf, z0, zd0, zf, zdf);
         transformedTrajectory.applyInverseTransform(transform);

         Point3D transformedz0 = new Point3D(z0);
         Vector3D transformedzd0 = new Vector3D(zd0);
         Point3D transformedzf = new Point3D(zf);
         Vector3D transformedzdf = new Vector3D(zdf);

         transformedz0.applyInverseTransform(transform);
         transformedzd0.applyInverseTransform(transform);
         transformedzf.applyInverseTransform(transform);
         transformedzdf.applyInverseTransform(transform);

         transformedTrajectory.compute(t0);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(transformedz0, transformedTrajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(transformedzd0, transformedTrajectory.getVelocity(), SMALL_EPSILON);

         transformedTrajectory.compute(tf);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(transformedzf, transformedTrajectory.getPosition(), SMALL_EPSILON);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(transformedzdf, transformedTrajectory.getVelocity(), SMALL_EPSILON);

         trajectoryWithTransformedInputs.setCubic(t0, tf, transformedz0, transformedzd0, transformedzf, transformedzdf);

         double dt = 1.0e-3;

         for (double t = t0; t <= tf; t += dt)
         {
            transformedTrajectory.compute(t);
            trajectoryWithTransformedInputs.compute(t);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectoryWithTransformedInputs.getPosition(), transformedTrajectory.getPosition(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectoryWithTransformedInputs.getVelocity(), transformedTrajectory.getVelocity(), SMALL_EPSILON);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectoryWithTransformedInputs.getAcceleration(), transformedTrajectory.getAcceleration(), SMALL_EPSILON);

         }
      }
   }

}