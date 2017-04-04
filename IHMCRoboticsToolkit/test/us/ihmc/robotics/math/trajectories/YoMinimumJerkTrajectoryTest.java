package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoMinimumJerkTrajectoryTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.6)
	@Test(timeout = 30000)
   public void testRandomInitialFinalConditions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");

      YoMinimumJerkTrajectory minimumJerkTrajectory = new YoMinimumJerkTrajectory("test", registry);


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
         double t0 = randomBetween(-10.0, 10.0);
         double tf = t0 + randomBetween(0.1, 10.0);

         minimumJerkTrajectory.setParams(x0, v0, a0, xf, vf, af, t0, tf);

         minimumJerkTrajectory.computeTrajectory(randomBetween(-10.0, 10.0));

         minimumJerkTrajectory.computeTrajectory(t0);
         assertEquals(x0, minimumJerkTrajectory.getPosition(), epsilon);
         assertEquals(v0, minimumJerkTrajectory.getVelocity(), epsilon);
         assertEquals(a0, minimumJerkTrajectory.getAcceleration(), epsilon);

         minimumJerkTrajectory.computeTrajectory(tf);
         assertEquals(xf, minimumJerkTrajectory.getPosition(), epsilon);
         assertEquals(vf, minimumJerkTrajectory.getVelocity(), epsilon);
         assertEquals(af, minimumJerkTrajectory.getAcceleration(), epsilon);

//       System.out.println("i=" + i);
      }
   }

   private double randomBetween(double min, double max)
   {
      return min + Math.random() * (max - min);
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testTimeExtensionRuntime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestMinimumJerkTrajectory");
      YoMinimumJerkTrajectory minimumJerkTrajectory = new YoMinimumJerkTrajectory("tester", registry);
      
      long starttime, endtime;
      double t = 0.0;
      double maxV = 10;
      double maxA = 10;
      int runs = 10000;

      starttime = System.nanoTime();

      Random random = new Random(107L);
      for (int i = 0; i < runs * 6; i++)
      {
         minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, random.nextDouble() * 0.5 + 0.75, 0.0, 0.0, 0.0, 0.5);
         minimumJerkTrajectory.timeExtension(t, maxV, maxA, true);
      }

      endtime = System.nanoTime();
      double runtime = (endtime - starttime) / runs * 1.0e-6;
      System.out.println("TestMinimumJerkTrajectory.testTimeExtensionRuntime: Execution Time = " + (runtime) + "ms per 6 calls");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindMaxVals()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestMinimumJerkTrajectory");
      YoMinimumJerkTrajectory minimumJerkTrajectory = new YoMinimumJerkTrajectory("tester", registry);
      
      double t = 0.0;
      double[] maximums = new double[2];
      double maxVexpected = 5.625;
      double maxAexpected = 34.641;
      double percent = 0.01;
      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.5);
      minimumJerkTrajectory.findMaxVelocityAndAccel(t, maximums);

      if (!MathTools.percentEquals(maxVexpected, maximums[0], percent))
      {
         throw new RuntimeException("TestMinimumJerkTrajectory.testFindMaxVals: Max velocity is wrong: expected=" + maxVexpected + ", actual=" + maximums[0]);
      }

      if (!MathTools.percentEquals(maxAexpected, maximums[1], percent))
      {
         throw new RuntimeException("TestMinimumJerkTrajectory.testFindMaxVals: Max accel is wrong: expected=" + maxAexpected + ", actual=" + maximums[1]);
      }
   }

	// FIXME That test is stuck in an infinite loop of some sort.
	@Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout=300000)
   public void testTimeExtension()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestMinimumJerkTrajectory");
      YoMinimumJerkTrajectory minimumJerkTrajectory = new YoMinimumJerkTrajectory("tester", registry);
      
      double t = 0.0;
      double maxV = 5.0;
      double maxA = 10.0;
      double expectedTime = 0.930605;
      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0);
      double newTime = minimumJerkTrajectory.timeExtension(t, maxV, maxA, true);
      if (!MathTools.percentEquals(expectedTime, newTime, 0.01))
      {
         throw new RuntimeException("TestMinimumJerkTrajectory.testTimeExtension: Returned desired time= " + newTime + ", expected= " + expectedTime);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testZeroLength()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestMinimumJerkTrajectory");
      YoMinimumJerkTrajectory minimumJerkTrajectory = new YoMinimumJerkTrajectory("tester", registry);
      
      double t = 0.5e-7;
      double[] vals = new double[3];
      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      minimumJerkTrajectory.computeTrajectoryDoubles(t, vals);

      if (Double.isNaN(vals[0]) || Double.isNaN(vals[1]) || Double.isNaN(vals[2]))
      {
         throw new RuntimeException("TestMinimumJerkTrajectory.testZeroLength: failed on zero displacement");
      }

      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0);
      minimumJerkTrajectory.computeTrajectoryDoubles(t, vals);

      if (Double.isNaN(vals[0]) || Double.isNaN(vals[1]) || Double.isNaN(vals[2]))
      {
         throw new RuntimeException("TestMinimumJerkTrajectory.testZeroLength: failed on zero time difference");
      }

      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.5);
      double Tf = minimumJerkTrajectory.timeExtension(t, 0.0, 0.0, true);
      System.out.println("TestMinimumJerkTrajectory.testZeroLength: Extending with maximums at 0.0: Tf = " + Tf);
      System.out.flush();

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBadInitialParams()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestMinimumJerkTrajectory");
      YoMinimumJerkTrajectory minimumJerkTrajectory = new YoMinimumJerkTrajectory("tester", registry);
      
      double maxV = 10;
      double maxA = 50;
      minimumJerkTrajectory.setParams(0.0, maxV * 1.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05);
      double newTime = minimumJerkTrajectory.timeExtension(1e-7, maxV, maxA, true);
      if (newTime > 100000)
      {
         throw new RuntimeException("TestMinimumJerkTrajectory.testBadInitialParams: cannot find suitable time");
      }
      else
      {
         System.out.println("TestMinimumJerkTrajectory.testBadInitialParams: Tf = " + newTime);
      }
   }
}
