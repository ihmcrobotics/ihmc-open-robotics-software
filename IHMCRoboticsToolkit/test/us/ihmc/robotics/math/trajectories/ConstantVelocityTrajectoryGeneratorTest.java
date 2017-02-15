package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class ConstantVelocityTrajectoryGeneratorTest
{
   private final double epsilon = 1e-7;
   private ConstantVelocityTrajectoryGenerator constantVelocityTrajectoryGenerator;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleTrajectory()
   {
      DoubleProvider initialPosition = new ConstantDoubleProvider(0.0);
      DoubleProvider velocity = new ConstantDoubleProvider(1.0);
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(5.0);
      
      ConstantVelocityTrajectoryGenerator simpleLinearTrajectory = new ConstantVelocityTrajectoryGenerator("", initialPosition, velocity, trajectoryTimeProvider, new YoVariableRegistry(""));
      simpleLinearTrajectory.initialize();
      
      simpleLinearTrajectory.compute(2.5);
      assertTrue(simpleLinearTrajectory.getValue() == 2.5);
      assertTrue(simpleLinearTrajectory.getVelocity() == 1.0);
      assertTrue(simpleLinearTrajectory.getAcceleration() == 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
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
         
         constantVelocityTrajectoryGenerator = new ConstantVelocityTrajectoryGenerator("", initialPosition, velocity, trajectoryTime, new YoVariableRegistry(""));
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
