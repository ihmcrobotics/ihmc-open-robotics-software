package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ConstantVelocityTrajectoryGeneratorTest
{
   private final double epsilon = 1e-7;
   private ConstantVelocityTrajectoryGenerator constantVelocityTrajectoryGenerator;

	@Test
   public void testSimpleTrajectory()
   {
      DoubleProvider initialPosition = new ConstantDoubleProvider(0.0);
      DoubleProvider velocity = new ConstantDoubleProvider(1.0);
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(5.0);
      
      ConstantVelocityTrajectoryGenerator simpleLinearTrajectory = new ConstantVelocityTrajectoryGenerator("", initialPosition, velocity, trajectoryTimeProvider, new YoRegistry("Dummy"));
      simpleLinearTrajectory.initialize();
      
      simpleLinearTrajectory.compute(2.5);
      assertTrue(simpleLinearTrajectory.getValue() == 2.5);
      assertTrue(simpleLinearTrajectory.getVelocity() == 1.0);
      assertTrue(simpleLinearTrajectory.getAcceleration() == 0.0);
   }

	@Test
   public void testRandomTrajectories()
   {
      final double numberOfIterations = 100;
      final double minimumTrajectoryTime = 1e-3;
      Random random = new Random(499300L);
      
      for(int i = 0; i < numberOfIterations; i++)
      {
         DoubleProvider initialPosition = new ConstantDoubleProvider(2 * random.nextDouble() - 1);
         DoubleProvider velocity = new ConstantDoubleProvider(2 * random.nextDouble() - 1);
         DoubleProvider trajectoryTime = new ConstantDoubleProvider(random.nextDouble() + minimumTrajectoryTime);
         
         constantVelocityTrajectoryGenerator = new ConstantVelocityTrajectoryGenerator("", initialPosition, velocity, trajectoryTime, new YoRegistry("Dummy"));
         constantVelocityTrajectoryGenerator.initialize();
         
         double randomTimeDuringTrajectory = trajectoryTime.getValue() * random.nextDouble();
         double expectedPosition = initialPosition.getValue() + velocity.getValue() * randomTimeDuringTrajectory;
         checkTrajectoryPositionVelocityAndAcceleration(randomTimeDuringTrajectory, expectedPosition, velocity.getValue());
         assertFalse(constantVelocityTrajectoryGenerator.isDone());

         double randomTimeBeforeTrajectory = - random.nextDouble();
         constantVelocityTrajectoryGenerator.compute(randomTimeBeforeTrajectory);
         assertFalse(constantVelocityTrajectoryGenerator.isDone());
         
         double randomTimeAfterTrajectory = trajectoryTime.getValue() + random.nextDouble() + epsilon;
         constantVelocityTrajectoryGenerator.compute(randomTimeAfterTrajectory);
         assertTrue(constantVelocityTrajectoryGenerator.isDone());
      }
   }
   
   private void checkTrajectoryPositionVelocityAndAcceleration(double time, double expectedPosition, double expectedVelocity)
   {
      constantVelocityTrajectoryGenerator.compute(time);
      assertEquals(constantVelocityTrajectoryGenerator.getValue(), expectedPosition, epsilon);
      assertEquals(constantVelocityTrajectoryGenerator.getVelocity(), expectedVelocity, epsilon);
      assertEquals(constantVelocityTrajectoryGenerator.getAcceleration(), 0.0, epsilon);
   }
}
