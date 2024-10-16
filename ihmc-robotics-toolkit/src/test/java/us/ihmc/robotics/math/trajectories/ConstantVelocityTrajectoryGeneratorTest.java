package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.trajectories.ConstantVelocityTrajectoryGenerator;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ConstantVelocityTrajectoryGeneratorTest
{
   private final double epsilon = 1e-7;
   private ConstantVelocityTrajectoryGenerator constantVelocityTrajectoryGenerator;

	@Test
   public void testSimpleTrajectory()
   {
      DoubleProvider initialPosition = () -> 0.0;
      DoubleProvider velocity = () -> 1.0;
      DoubleProvider trajectoryTimeProvider = () -> 5.0;
      
      ConstantVelocityTrajectoryGenerator simpleLinearTrajectory = new ConstantVelocityTrajectoryGenerator("", initialPosition, velocity, trajectoryTimeProvider, new YoRegistry("Dummy"));
      simpleLinearTrajectory.initialize();
      
      simpleLinearTrajectory.compute(2.5);
      assertEquals(2.5, simpleLinearTrajectory.getValue(), epsilon);
      assertEquals(1.0, simpleLinearTrajectory.getVelocity(), epsilon);
      assertEquals(0.0, simpleLinearTrajectory.getAcceleration(), epsilon);
   }

	@Test
   public void testRandomTrajectories()
   {
      final double numberOfIterations = 100;
      final double minimumTrajectoryTime = 1e-3;
      Random random = new Random(499300L);
      
      for(int i = 0; i < numberOfIterations; i++)
      {
         double position = 2 * random.nextDouble() - 1;
         double velocity = 2 * random.nextDouble() - 1;
         double trajectoryTime = random.nextDouble() + minimumTrajectoryTime;
         DoubleProvider positionProvider = () -> position;
         DoubleProvider velocityProvider = () -> velocity;
         DoubleProvider trajectoryTimeProvider = () -> trajectoryTime;
         
         constantVelocityTrajectoryGenerator = new ConstantVelocityTrajectoryGenerator("", positionProvider, velocityProvider, trajectoryTimeProvider, new YoRegistry("Dummy"));
         constantVelocityTrajectoryGenerator.initialize();

         constantVelocityTrajectoryGenerator.compute(0.0);
         assertEquals(positionProvider.getValue(), constantVelocityTrajectoryGenerator.getValue(), epsilon);
         
         double randomTimeDuringTrajectory = trajectoryTimeProvider.getValue() * random.nextDouble();
         double expectedPosition = positionProvider.getValue() + velocityProvider.getValue() * randomTimeDuringTrajectory;

         checkTrajectoryPositionVelocityAndAcceleration(randomTimeDuringTrajectory, expectedPosition, velocityProvider.getValue());
         assertFalse(constantVelocityTrajectoryGenerator.isDone());

         double randomTimeBeforeTrajectory = - random.nextDouble();
         constantVelocityTrajectoryGenerator.compute(randomTimeBeforeTrajectory);
         assertFalse(constantVelocityTrajectoryGenerator.isDone());
         
         double randomTimeAfterTrajectory = trajectoryTimeProvider.getValue() + random.nextDouble() + epsilon;
         constantVelocityTrajectoryGenerator.compute(randomTimeAfterTrajectory);
         assertTrue(constantVelocityTrajectoryGenerator.isDone());
      }
   }
   
   private void checkTrajectoryPositionVelocityAndAcceleration(double time, double expectedPosition, double expectedVelocity)
   {
      constantVelocityTrajectoryGenerator.compute(time);
      assertEquals("Position is wrong ", constantVelocityTrajectoryGenerator.getValue(), expectedPosition, epsilon);
      assertEquals("Velocity is wrong ", constantVelocityTrajectoryGenerator.getVelocity(), expectedVelocity, epsilon);
      assertEquals("Acceleration is wrong ", constantVelocityTrajectoryGenerator.getAcceleration(), 0.0, epsilon);
   }
}
