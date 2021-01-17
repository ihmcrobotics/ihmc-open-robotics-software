package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Assertions;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial;

public class PolynomialTest
{
   private static final int ITERATIONS = 1000;
   private static final double LARGE_EPSILON = 1.0e-2;
   private static final double SMALL_EPSILON = 1.0e-12;

   String namePrefix = "TrajectoryTest";
   private final Random random = new Random(3294508L);

   @Test
   public void testLinearSet()
   {
      Polynomial traj = new Polynomial(2);
      assertEquals(0, traj.getNumberOfCoefficients());
      traj.setLinear(1, 2, 3, 5);
      assertEquals(1, traj.getInitialTime(), SMALL_EPSILON);
      assertEquals(2, traj.getFinalTime(), SMALL_EPSILON);
      traj.compute(traj.getInitialTime());
      assertEquals(3.0, traj.getValue(), SMALL_EPSILON);
      traj.compute(traj.getFinalTime());
      assertEquals(5.0, traj.getValue(), SMALL_EPSILON);
      assertEquals(2, traj.getCoefficient(1), SMALL_EPSILON);
      assertEquals(1, traj.getCoefficient(0), SMALL_EPSILON);
   }

   @Test
   public void testSetConstant() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         Polynomial trajectory = new Polynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + random.nextDouble();
         double z = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 1)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setConstant(t0, tf, z));
            continue;
         }

         trajectory.setConstant(t0, tf, z);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            assertEquals(z, trajectory.getValue(), SMALL_EPSILON);
            assertEquals(0, trajectory.getVelocity(), SMALL_EPSILON);
            assertEquals(0, trajectory.getAcceleration(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetLinear() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         Polynomial trajectory = new Polynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + random.nextDouble();
         double z0 = RandomNumbers.nextDouble(random, 1.0);
         double zf = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 2)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setLinear(t0, tf, z0, zf));
            continue;
         }

         trajectory.setLinear(t0, tf, z0, zf);

         double zDot = (zf - z0) / (tf - t0);
         Polynomial derivative = new Polynomial(1);
         derivative.setConstant(t0, tf, zDot);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), SMALL_EPSILON);
         assertEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
         assertEquals(0, trajectory.getAcceleration(), SMALL_EPSILON);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), SMALL_EPSILON);
         assertEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
         assertEquals(0, trajectory.getAcceleration(), SMALL_EPSILON);

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            assertEquals(EuclidCoreTools.interpolate(z0, zf, (t - t0) / (tf - t0)), trajectory.getValue(), SMALL_EPSILON);
            assertEquals(zDot, trajectory.getVelocity(), SMALL_EPSILON);
            assertEquals(0, trajectory.getAcceleration(), SMALL_EPSILON);

            derivative.compute(t);
            assertEquals(derivative.getValue(), trajectory.getVelocity(), SMALL_EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetQuadratic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         Polynomial trajectory = new Polynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;
         double z0 = RandomNumbers.nextDouble(random, 1.0);
         double zd0 = RandomNumbers.nextDouble(random, 1.0);
         double zf = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 3)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setQuadratic(t0, tf, z0, zd0, zf));
            continue;
         }

         trajectory.setQuadratic(t0, tf, z0, zd0, zf);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), SMALL_EPSILON);
         assertEquals(zd0, trajectory.getVelocity(), SMALL_EPSILON);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), SMALL_EPSILON);

         Polynomial derivative = new Polynomial(2);
         derivative.setLinear(t0, tf, zd0, trajectory.getVelocity());

         double dt = 1.0e-8;

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            derivative.compute(t);

            assertEquals(derivative.getValue(), trajectory.getVelocity(), SMALL_EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);

            trajectory.compute(t + dt);
            double nextPosition = trajectory.getValue();
            trajectory.compute(t - dt);
            double lastPosition = trajectory.getValue();
            assertEquals(0.5 * (nextPosition - lastPosition) / dt, trajectory.getVelocity(), 1.0e-6);

         }
      }
   }

   @Test
   public void testSetCubic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = random.nextInt(10);
         Polynomial trajectory = new Polynomial(maxNumberOfCoefficients);
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;
         double z0 = RandomNumbers.nextDouble(random, 1.0);
         double zd0 = RandomNumbers.nextDouble(random, 1.0);
         double zf = RandomNumbers.nextDouble(random, 1.0);
         double zdf = RandomNumbers.nextDouble(random, 1.0);

         if (maxNumberOfCoefficients < 4)
         {
            Assertions.assertExceptionThrown(RuntimeException.class, () -> trajectory.setCubic(t0, tf, z0, zd0, zf, zdf));
            continue;
         }

         trajectory.setCubic(t0, tf, z0, zd0, zf, zdf);

         trajectory.compute(t0);
         assertEquals(z0, trajectory.getValue(), SMALL_EPSILON);
         assertEquals(zd0, trajectory.getVelocity(), SMALL_EPSILON);

         Polynomial derivative = new Polynomial(3);
         derivative.setQuadratic(t0, tf, zd0, trajectory.getAcceleration(), zdf);

         trajectory.compute(tf);
         assertEquals(zf, trajectory.getValue(), SMALL_EPSILON);

         double dt = 1.0e-8;

         for (double t = t0; t <= tf; t += (tf - t0) / 1000)
         {
            trajectory.compute(t);
            derivative.compute(t);

            assertEquals(derivative.getValue(), trajectory.getVelocity(), SMALL_EPSILON);
            assertEquals(derivative.getVelocity(), trajectory.getAcceleration(), SMALL_EPSILON);

            trajectory.compute(t + dt);
            double nextPosition = trajectory.getValue();
            trajectory.compute(t - dt);
            double lastPosition = trajectory.getValue();
            assertEquals(0.5 * (nextPosition - lastPosition) / dt, trajectory.getVelocity(), 5.0e-6);

         }
      }
   }






   @Test
   public void testQuinticTrajectory()
   {
      Polynomial quinticTrajectory = new Polynomial(-10.0, 10.0, new double[] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

      quinticTrajectory.setQuintic(0.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

      quinticTrajectory.compute(0.0);
      assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getValue(), 4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);

      quinticTrajectory.setQuintic(-1.0, 1.0, 1.0, -2.0, 3.0, -4.0, -5.0, 6.0);

      quinticTrajectory.compute(-1.0);
      assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getValue(), -4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);
   }

   private static int getCoefficientMultiplierForDerivative(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent; i > exponent - order; i--)
      {
         coeff *= i;
      }
      return coeff;
   }
}