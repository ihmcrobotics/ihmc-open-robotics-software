package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MultipleWaypointsTrajectoryGeneratorTest
{

   private final double EPSILON = 1e-3;

   @Test
   public void test()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      double trajectoryTime = 1.0;
      double dt = 0.001;

      CubicPolynomialTrajectoryGenerator simpleTrajectory;
      MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectory;

      YoVariableDoubleProvider trajectoryTimeProvider;

      trajectoryTimeProvider = new YoVariableDoubleProvider("trajectoryTime", registry);
      trajectoryTimeProvider.set(trajectoryTime);

      DoubleProvider initialPositionProvider = new ConstantDoubleProvider(0.0);
      DoubleProvider finalPositionProvider = new ConstantDoubleProvider(1.0);

      simpleTrajectory = new CubicPolynomialTrajectoryGenerator("simpleTraj", initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, registry);
      simpleTrajectory.initialize();

      int numberOfWaypoints = 11;

      multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", 15, registry);
      multipleWaypointsTrajectory.clear();
           
      for (int i = 0; i < numberOfWaypoints; i++)
      {

         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         double waypointPosition = simpleTrajectory.getValue();
         double waypointVelocity = simpleTrajectory.getVelocity();
         
         multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);

      }
      
      multipleWaypointsTrajectory.initialize();

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         multipleWaypointsTrajectory.compute(t);
         simpleTrajectory.compute(t);
         
         assertEquals(simpleTrajectory.getValue(), multipleWaypointsTrajectory.getValue(), EPSILON );
         assertEquals(simpleTrajectory.getVelocity(), multipleWaypointsTrajectory.getVelocity(), EPSILON );
         assertEquals(simpleTrajectory.getAcceleration(), multipleWaypointsTrajectory.getAcceleration(), EPSILON);
      }
   }
   
   @Test
   public void testPassingThroughWayPoints()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      int numberOfWaypoints = 11;

      MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", 15, registry);
      multipleWaypointsTrajectory.clear();
      
      Random random = new Random(10L);
      double trajectoryTime = 1.0;
      
      double[] positions = new double[numberOfWaypoints];
      double[] velocities = new double[numberOfWaypoints];
      
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         double waypointPosition = random.nextDouble();
         positions[i] = waypointPosition;
         
         double waypointVelocity = random.nextDouble();
         velocities[i] = waypointVelocity;
         
         multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);
      }
      
      multipleWaypointsTrajectory.initialize();
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double time = i * trajectoryTime / (numberOfWaypoints - 1.0);
         multipleWaypointsTrajectory.compute(time);
         
         assertEquals(positions[i], multipleWaypointsTrajectory.getValue(), EPSILON );
         assertEquals(velocities[i], multipleWaypointsTrajectory.getVelocity(), EPSILON );
      }
   }

   @Test
   public void testOneWaypoint()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      int maxNumberOfWaypoints = 5;
      MultipleWaypointsTrajectoryGenerator trajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", maxNumberOfWaypoints, registry);
      trajectory.clear();

      double timeAtWaypoint = 0.0406;
      double positionAtWaypoint = 0.47;
      double velocityAtWaypoint = 0.10;
      trajectory.appendWaypoint(timeAtWaypoint, positionAtWaypoint, velocityAtWaypoint);
      trajectory.initialize();

      trajectory.compute(0.036);
      assertEquals(positionAtWaypoint, trajectory.getValue());
      assertEquals(0.0, trajectory.getVelocity());
      assertEquals(0.0, trajectory.getAcceleration());
      
      trajectory.compute(0.042);
      assertEquals(positionAtWaypoint, trajectory.getValue());
      assertEquals(0.0, trajectory.getVelocity());
      assertEquals(0.0, trajectory.getAcceleration());

      trajectory.compute(timeAtWaypoint);
      assertEquals(positionAtWaypoint, trajectory.getValue());
      assertEquals(velocityAtWaypoint, trajectory.getVelocity());
      assertEquals(0.0, trajectory.getAcceleration());
   }
}
