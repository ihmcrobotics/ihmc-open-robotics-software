package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class ConstantAccelerationTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefix";
   private YoVariableRegistry parentRegistry = new YoVariableRegistry("parentRegistry");

   private DoubleProvider initialPositionProvider;
   private DoubleProvider finalPositionProvider;
   private DoubleProvider initialVelocityProvider;
   private DoubleProvider trajectoryTimeProvider;

   private static final double timeRequired = 10.0;
   private static final double INITIALVELOCITY = Math.random();
   private static final double EPSILON = 1e-10;
   private static final double CONSTANT = Math.random();
   private static final double DT = timeRequired / 10.0;

   private ConstantAccelerationTrajectoryGenerator generator;

   @Before
   public void setUp()
   {
      initialPositionProvider = new ConstantDoubleProvider(0.0);
      trajectoryTimeProvider = new ConstantDoubleProvider(timeRequired);
      initialVelocityProvider = new ConstantDoubleProvider(INITIALVELOCITY);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructor()
   {
      finalPositionProvider = new ConstantDoubleProvider(CONSTANT * timeRequired * timeRequired + INITIALVELOCITY * timeRequired);

      generator = null;

      try
      {
         generator = new ConstantAccelerationTrajectoryGenerator(namePrefix, initialPositionProvider, finalPositionProvider, initialVelocityProvider,
               trajectoryTimeProvider, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsDone()
   {
      finalPositionProvider = new ConstantDoubleProvider(CONSTANT * timeRequired * timeRequired + INITIALVELOCITY * timeRequired);

      generator = new ConstantAccelerationTrajectoryGenerator(namePrefix, initialPositionProvider, finalPositionProvider, initialVelocityProvider,
            trajectoryTimeProvider, parentRegistry);

      generator.initialize();
      generator.compute(trajectoryTimeProvider.getValue());
      assertFalse(generator.isDone());

      generator.compute(trajectoryTimeProvider.getValue() + EPSILON);
      assertTrue(generator.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //s = c * t ^ 2 + v0 * t
   public void testIncreasing()
   {
      finalPositionProvider = new ConstantDoubleProvider(CONSTANT * timeRequired * timeRequired + INITIALVELOCITY * timeRequired);

      generator = new ConstantAccelerationTrajectoryGenerator(namePrefix, initialPositionProvider, finalPositionProvider, initialVelocityProvider,
            trajectoryTimeProvider, parentRegistry);

      generator.initialize();
      for (double t = 0; t < timeRequired; t += DT)
      {
         generator.compute(t);

         assertEquals(CONSTANT * t * t + INITIALVELOCITY * t, generator.getValue(), EPSILON);
         assertEquals(2.0 * CONSTANT * t + INITIALVELOCITY, generator.getVelocity(), EPSILON);
         assertEquals(2.0 * CONSTANT, generator.getAcceleration(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   //s = -c * t ^ 2 + v0 * t
   public void testDecreasing()
   {
      finalPositionProvider = new ConstantDoubleProvider(-CONSTANT * timeRequired * timeRequired + INITIALVELOCITY * timeRequired);

      generator = new ConstantAccelerationTrajectoryGenerator(namePrefix, initialPositionProvider, finalPositionProvider, initialVelocityProvider,
            trajectoryTimeProvider, parentRegistry);

      generator.initialize();
      for (double t = 0; t < timeRequired; t += DT)
      {
         generator.compute(t);

         assertEquals(-CONSTANT * t * t + INITIALVELOCITY * t, generator.getValue(), EPSILON);
         assertEquals(-2.0 * CONSTANT * t + INITIALVELOCITY, generator.getVelocity(), EPSILON);
         assertEquals(-2.0 * CONSTANT, generator.getAcceleration(), EPSILON);
      }
   }
}