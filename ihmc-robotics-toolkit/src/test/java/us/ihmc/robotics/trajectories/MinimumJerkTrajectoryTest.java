package us.ihmc.robotics.trajectories;

import static junit.framework.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class MinimumJerkTrajectoryTest
{
   public MinimumJerkTrajectoryTest()
   {
   }


   private double randomBetween(double min, double max)
   {
      return min + Math.random() * (max - min);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testRandomInitialFinalConditions()
   {
      MinimumJerkTrajectory minimumJerkTrajectory = new MinimumJerkTrajectory();


      int numberOfTests = 1000000;
      double epsilon = 1e-3;

      for (int i = 0; i < numberOfTests; i++)
      {
         double x0 = randomBetween(-10.0, 10.0);
         double v0 = randomBetween(-10.0, 10.0);
         double a0 = randomBetween(-10.0, 10.0);
         double xf = randomBetween(-10.0, 10.0);
         double vf = randomBetween(-10.0, 10.0);
         double af = randomBetween(-10.0, 10.0);

         double moveDuration = randomBetween(0.1, 10.0);

         minimumJerkTrajectory.setMoveParameters(x0, v0, a0, xf, vf, af, moveDuration);

         minimumJerkTrajectory.computeTrajectory(randomBetween(-10.0, 10.0));

         minimumJerkTrajectory.computeTrajectory(0.0);
         assertEquals(x0, minimumJerkTrajectory.getPosition(), epsilon);
         assertEquals(v0, minimumJerkTrajectory.getVelocity(), epsilon);
         assertEquals(a0, minimumJerkTrajectory.getAcceleration(), epsilon);

         minimumJerkTrajectory.computeTrajectory(moveDuration);
         assertEquals(xf, minimumJerkTrajectory.getPosition(), epsilon);
         assertEquals(vf, minimumJerkTrajectory.getVelocity(), epsilon);
         assertEquals(af, minimumJerkTrajectory.getAcceleration(), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCheckVelocityAndAcceleration()
   {
      double x0 = 0.0;
      double v0 = 0.0;
      double a0 = 0.0;
      double xf = 1.0;
      double vf = 0.0;
      double af = 0.0;

      double moveDuration = 2.0;
      MinimumJerkTrajectory minimumJerkTrajectory = new MinimumJerkTrajectory();

      minimumJerkTrajectory.setMoveParameters(x0, v0, a0, xf, vf, af, moveDuration);

      double epsilon = 1e-6;

      minimumJerkTrajectory.computeTrajectory(0.0);
      assertEquals(x0, minimumJerkTrajectory.getPosition(), epsilon);
      assertEquals(v0, minimumJerkTrajectory.getVelocity(), epsilon);
      assertEquals(a0, minimumJerkTrajectory.getAcceleration(), epsilon);

      minimumJerkTrajectory.computeTrajectory(moveDuration);
      assertEquals(xf, minimumJerkTrajectory.getPosition(), epsilon);
      assertEquals(vf, minimumJerkTrajectory.getVelocity(), epsilon);
      assertEquals(af, minimumJerkTrajectory.getAcceleration(), epsilon);
   }
}
