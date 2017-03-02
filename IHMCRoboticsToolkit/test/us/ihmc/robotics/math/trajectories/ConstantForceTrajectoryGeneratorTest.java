package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class ConstantForceTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefix";
   private YoVariableRegistry parentRegistry = new YoVariableRegistry("parentRegistry");
   private double force = 10.0;
   private double finalTime = 10.0;

   private static final double timeRequired = 10.0;
   private static final double EPSILON = 1e-14;
   private static final double DT = timeRequired / 10.0;

   private ConstantForceTrajectoryGenerator generator;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructor()
   {
      generator = null;

      try
      {
         generator = new ConstantForceTrajectoryGenerator(namePrefix, force, finalTime, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

      finalTime = -1.0;

      try
      {
         generator = new ConstantForceTrajectoryGenerator(namePrefix, force, finalTime, parentRegistry);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsDone()
   {
      generator = new ConstantForceTrajectoryGenerator(namePrefix, force, finalTime, parentRegistry);

      generator.initialize();
      generator.compute(finalTime);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIncreasing()
   {
      generator = new ConstantForceTrajectoryGenerator(namePrefix, force, finalTime, parentRegistry);

      generator.initialize();
      for (double t = 0; t < timeRequired; t += DT)
      {
         generator.compute(t);

         assertEquals(force, generator.getValue(), EPSILON);
         assertEquals(0.0, generator.getVelocity(), EPSILON);
         assertEquals(0.0, generator.getAcceleration(), EPSILON);
      }
   }
}