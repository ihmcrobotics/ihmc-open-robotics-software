package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class MultipleWaypointsTrajectoryGeneratorTest
{

   private final double EPSILON = 1e-3;

   @Test
   public void test()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      double trajectoryTime = 1.0;
      double dt = 0.001;

      MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectory;

      YoDouble trajectoryTimeProvider;

      trajectoryTimeProvider = new YoDouble("trajectoryTime", registry);
      trajectoryTimeProvider.set(trajectoryTime);

      DoubleProvider initialPositionProvider = () -> 0.0;
      DoubleProvider finalPositionProvider = () -> 1.0;

      YoPolynomial simpleTrajectory = new YoPolynomial("simpleTraj", 4, registry);
      simpleTrajectory.setCubic(0.0, trajectoryTimeProvider.getDoubleValue(), initialPositionProvider.getValue(), finalPositionProvider.getValue());

      int numberOfWaypoints = 11;

      multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", 15, registry);
      multipleWaypointsTrajectory.clear();
           
      for (int i = 0; i < numberOfWaypoints; i++)
      {

         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         double waypointPosition = simpleTrajectory.getValue();
         double waypointVelocity = simpleTrajectory.getVelocity();
         double waypointAcceleration = simpleTrajectory.getAcceleration();

         multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity, waypointAcceleration);

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
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      int numberOfWaypoints = 11;

      MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", 15, registry);
      multipleWaypointsTrajectory.clear();
      
      Random random = new Random(10L);
      double trajectoryTime = 1.0;
      
      double[] positions = new double[numberOfWaypoints];
      double[] velocities = new double[numberOfWaypoints];
      double[] accelerations = new double[numberOfWaypoints];

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         double waypointPosition = random.nextDouble();
         positions[i] = waypointPosition;
         
         double waypointVelocity = random.nextDouble();
         velocities[i] = waypointVelocity;

         double waypointAcceleration = random.nextDouble();
         accelerations[i] = waypointAcceleration;
         
         multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity, waypointAcceleration);
      }
      
      multipleWaypointsTrajectory.initialize();
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double time = i * trajectoryTime / (numberOfWaypoints - 1.0);
         multipleWaypointsTrajectory.compute(time);
         
         assertEquals(positions[i], multipleWaypointsTrajectory.getValue(), EPSILON );
         assertEquals(velocities[i], multipleWaypointsTrajectory.getVelocity(), EPSILON );
         assertEquals(accelerations[i], multipleWaypointsTrajectory.getAcceleration(), EPSILON );
      }
   }

   @Test
   public void testOneWaypoint()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      int maxNumberOfWaypoints = 5;
      MultipleWaypointsTrajectoryGenerator trajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", maxNumberOfWaypoints, registry);
      trajectory.clear();

      double timeAtWaypoint = 0.0406;
      double positionAtWaypoint = 0.47;
      double velocityAtWaypoint = 0.10;
      double accelerationAtWaypoint = 0.20;
      trajectory.appendWaypoint(timeAtWaypoint, positionAtWaypoint, velocityAtWaypoint, accelerationAtWaypoint);
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

   @Test
   public void testEdgeCase()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      int maxNumberOfWaypoints = 5;
      MultipleWaypointsTrajectoryGenerator trajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", maxNumberOfWaypoints, registry);
      trajectory.clear();

      trajectory.appendWaypoint(0.0, 0.0, 0.0, 0.0);
      trajectory.appendWaypoint(0.5, 0.026337062843167836, 0.0, 0.0);
      trajectory.initialize();

      trajectory.compute(0.0);
      assertEquals(0.0, trajectory.getValue(), 1e-5);
      assertEquals(0.0, trajectory.getVelocity(), 1e-5);

      trajectory.compute(0.5);
      assertEquals(0.026337062843167836, trajectory.getValue(), 1e-5);
      assertEquals(0.0, trajectory.getVelocity(), 1e-5);
   }
}
