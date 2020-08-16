package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ConstantForceTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefix";
   private YoRegistry parentRegistry = new YoRegistry("parentRegistry");
   private double force = 10.0;
   private double finalTime = 10.0;

   private static final double timeRequired = 10.0;
   private static final double EPSILON = 1e-14;
   private static final double DT = timeRequired / 10.0;

   private ConstantForceTrajectoryGenerator generator;

	@Test
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

	@Test
   public void testIsDone()
   {
      generator = new ConstantForceTrajectoryGenerator(namePrefix, force, finalTime, parentRegistry);

      generator.initialize();
      generator.compute(finalTime);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@Test
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