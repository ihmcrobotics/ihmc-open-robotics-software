package us.ihmc.robotics.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.Fast})
public class MultipleWaypointsTrajectoryGeneratorTest
{

   private final double EPSILON = 1e-3;

   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 300000)
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

      multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", registry);
      multipleWaypointsTrajectory.clear();
           
      for (int i = 0; i < numberOfWaypoints; i++)
      {

         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         double waypointPosition = simpleTrajectory.getValue();
         double waypointVelocity = simpleTrajectory.getVelocity();
         
         if (i == 0)
         {
            multipleWaypointsTrajectory.setInitialCondition(waypointPosition, waypointVelocity);
         }
         else
         {
            multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);
         }

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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testPassingThroughWayPoints()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      int numberOfWaypoints = 11;

      MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", registry);
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
         
         if (i == 0)
         {
            multipleWaypointsTrajectory.setInitialCondition(waypointPosition, waypointVelocity);
         }
         else
         {
            multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);
         }
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testNotIncreasingTimes()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      double trajectoryTime = 1.0;

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

      multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", registry);
      multipleWaypointsTrajectory.clear();
          
      int[] indecies = new int[]{0, 4, 6, 2, 9, 5, 7, 1, 8, 10, 3};
      
      boolean success = true;
      for (int i = 0; i < numberOfWaypoints; i++)
      {

         double timeAtWaypoint = indecies[i] * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         double waypointPosition = simpleTrajectory.getValue();
         double waypointVelocity = simpleTrajectory.getVelocity();
         
         if (timeAtWaypoint == 0.0)
         {
            multipleWaypointsTrajectory.setInitialCondition(waypointPosition, waypointVelocity);
         }
         else
         {
            success &= multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);
         }
      }
      
      assertFalse(success);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testNotInitialized()
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

      multipleWaypointsTrajectory = new MultipleWaypointsTrajectoryGenerator("testedTraj", registry);
      multipleWaypointsTrajectory.clear();
           
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         double waypointPosition = simpleTrajectory.getValue();
         double waypointVelocity = simpleTrajectory.getVelocity();
         
         if (i == 0)
         {
            multipleWaypointsTrajectory.setInitialCondition(waypointPosition, waypointVelocity);
         }
         else
         {
            multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);
         }
      }

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         multipleWaypointsTrajectory.compute(t);
         simpleTrajectory.compute(t);
         
         assertEquals(simpleTrajectory.getValue(), multipleWaypointsTrajectory.getValue(), EPSILON );
         assertEquals(simpleTrajectory.getVelocity(), multipleWaypointsTrajectory.getVelocity(), EPSILON );
         assertEquals(simpleTrajectory.getAcceleration(), multipleWaypointsTrajectory.getAcceleration(), EPSILON);
      }
      
      
      // Test robustness to creating multiple trajectories
      trajectoryTime = 4.0;
      trajectoryTimeProvider.set(trajectoryTime);

      initialPositionProvider = new ConstantDoubleProvider(6.0);
      finalPositionProvider = new ConstantDoubleProvider(10.5);
      
      simpleTrajectory.initialize();

      numberOfWaypoints = 8;
      multipleWaypointsTrajectory.clear();
      
      for (int i = 0; i < numberOfWaypoints; i++)
      {

         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         double waypointPosition = simpleTrajectory.getValue();
         double waypointVelocity = simpleTrajectory.getVelocity();
         
         if (i == 0)
         {
            multipleWaypointsTrajectory.setInitialCondition(waypointPosition, waypointVelocity);
         }
         else
         {
            multipleWaypointsTrajectory.appendWaypoint(timeAtWaypoint , waypointPosition, waypointVelocity);
         }
      }

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         multipleWaypointsTrajectory.compute(t);
         simpleTrajectory.compute(t);
         
         assertEquals(simpleTrajectory.getValue(), multipleWaypointsTrajectory.getValue(), EPSILON );
         assertEquals(simpleTrajectory.getVelocity(), multipleWaypointsTrajectory.getVelocity(), EPSILON );
         assertEquals(simpleTrajectory.getAcceleration(), multipleWaypointsTrajectory.getAcceleration(), EPSILON);
      }
   }
}
