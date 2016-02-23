package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import javax.ws.rs.Path;

@DeployableTestClass(targets = {TestPlanTarget.Fast}) public class PositionPathPlanningTrajectoryGeneratorTest
{

   private final double EPSILON = 1e-2;

   @DeployableTestMethod(estimatedDuration = 0.1) @Test(timeout = 300000) public void testWithDefaultSettings()
   {
      YoVariableRegistry registry = new YoVariableRegistry("traj");

      double trajectoryTime = 1.0;
      double dt = 0.001;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      PositionPathPlanningTrajectoryGenerator PathPlanningTraj;
      StraightLinePositionTrajectoryGenerator simpleTrajectory;

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 1.0, 0.0, 1.0));
      PositionProvider finalPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 0.2, 1.0, 0.4));
      simpleTrajectory = new StraightLinePositionTrajectoryGenerator("simpleTraj", worldFrame, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      simpleTrajectory.initialize();

      int numberOfWaypoints = 11;
      PathPlanningTraj = new PositionPathPlanningTrajectoryGenerator("testedTraj", 50, worldFrame, registry);
      PathPlanningTraj.clear();

      FramePoint waypointPosition = new FramePoint();
      FrameVector waypointVelocity = new FrameVector();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         simpleTrajectory.getPosition(waypointPosition);
         simpleTrajectory.getVelocity(waypointVelocity);
         PathPlanningTraj.appendWaypoint(timeAtWaypoint, waypointPosition);
      }
      PathPlanningTraj.initialize();

      FramePoint positionToPackMultiple = new FramePoint(worldFrame);
      FrameVector velocityToPackMultiple = new FrameVector(worldFrame);
      FrameVector accelerationToPackMultiple = new FrameVector(worldFrame);

      FramePoint positionToPackSimple = new FramePoint(worldFrame);
      FrameVector velocityToPackSimple = new FrameVector(worldFrame);
      FrameVector accelerationToPackSimple = new FrameVector(worldFrame);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         PathPlanningTraj.compute(t);
         PathPlanningTraj.getLinearData(positionToPackMultiple, velocityToPackMultiple, accelerationToPackMultiple);

         simpleTrajectory.compute(t);
         simpleTrajectory.getLinearData(positionToPackSimple, velocityToPackSimple, accelerationToPackSimple);

         boolean positionEqual = positionToPackMultiple.epsilonEquals(positionToPackSimple, EPSILON);
         assertTrue(positionEqual);

         boolean velocityEqual = velocityToPackMultiple.epsilonEquals(velocityToPackSimple, 5.0 * EPSILON);
         assertTrue(velocityEqual);

      }

   }

   @DeployableTestMethod(estimatedDuration = 0.1) @Test(timeout = 300000) public void testTimeGeneration()
   {
      YoVariableRegistry registry = new YoVariableRegistry("traj");

      double trajectoryTime = 1.0;
      double dt = 0.001;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      PositionPathPlanningTrajectoryGenerator PathPlanningTraj;
      StraightLinePositionTrajectoryGenerator simpleTrajectory;

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 0.0, 0.0, 0.0));
      PositionProvider finalPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 1.0, 1.0, 1.0));
      simpleTrajectory = new StraightLinePositionTrajectoryGenerator("simpleTraj", worldFrame, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      simpleTrajectory.initialize();

      int numberOfWaypoints = 11;
      PathPlanningTraj = new PositionPathPlanningTrajectoryGenerator("testedTraj", 50, worldFrame, registry);
      PathPlanningTraj.clear();
      PathPlanningTraj.activateWaypointTimeGeneration(0, trajectoryTime);

      FramePoint waypointPosition = new FramePoint();
      FrameVector waypointVelocity = new FrameVector();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         simpleTrajectory.getPosition(waypointPosition);

         PathPlanningTraj.appendWaypoint(waypointPosition);
      }
      PathPlanningTraj.initialize();

      FramePoint positionToPackMultiple = new FramePoint(worldFrame);
      FrameVector velocityToPackMultiple = new FrameVector(worldFrame);
      FrameVector accelerationToPackMultiple = new FrameVector(worldFrame);

      FramePoint tempPoint = new FramePoint(worldFrame);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         PathPlanningTraj.compute(t);
         PathPlanningTraj.getLinearData(positionToPackMultiple, velocityToPackMultiple, accelerationToPackMultiple);

         if (t > 10 * dt && t < (trajectoryTime - 10 * dt))
         {
            tempPoint.set(t, t, t);
            boolean positionEquals = positionToPackMultiple.epsilonEquals(tempPoint, EPSILON);
            assertTrue(positionEquals);

            boolean velocityEqual = velocityToPackMultiple.epsilonEquals(new Vector3d(1.0, 1.0, 1.0), 5.0 * EPSILON);
            assertTrue(velocityEqual);
         }
      }

   }

   @DeployableTestMethod(estimatedDuration = 0.1) @Test(timeout = 300000) public void testWithInitialAndFinalVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("traj");

      double trajectoryTime = 1.0;
      double dt = 0.001;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      PositionPathPlanningTrajectoryGenerator PathPlanningTraj;
      StraightLinePositionTrajectoryGenerator simpleTrajectory;

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 1.0, 0.0, 1.0));
      PositionProvider finalPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 0.2, 1.0, 0.4));
      simpleTrajectory = new StraightLinePositionTrajectoryGenerator("simpleTraj", worldFrame, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      simpleTrajectory.initialize();

      int numberOfWaypoints = 11;
      PathPlanningTraj = new PositionPathPlanningTrajectoryGenerator("testedTraj", 50, worldFrame, registry);
      PathPlanningTraj.clear();

      FramePoint waypointPosition = new FramePoint();
      FrameVector waypointVelocity = new FrameVector();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         simpleTrajectory.getPosition(waypointPosition);
         simpleTrajectory.getVelocity(waypointVelocity);
         PathPlanningTraj.appendWaypoint(timeAtWaypoint, waypointPosition);
      }
      PathPlanningTraj.setInitialVelocity(new Vector3d(0.1, 0.2, 0.3));
      PathPlanningTraj.setFinalVelocity(new Vector3d(0.1, 0.2, 0.3));
      PathPlanningTraj.initialize();

      FramePoint positionToPackMultiple = new FramePoint(worldFrame);
      FrameVector velocityToPackMultiple = new FrameVector(worldFrame);
      FrameVector accelerationToPackMultiple = new FrameVector(worldFrame);

      FramePoint positionToPackSimple = new FramePoint(worldFrame);
      FrameVector velocityToPackSimple = new FrameVector(worldFrame);
      FrameVector accelerationToPackSimple = new FrameVector(worldFrame);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         PathPlanningTraj.compute(t);
         PathPlanningTraj.getLinearData(positionToPackMultiple, velocityToPackMultiple, accelerationToPackMultiple);

         simpleTrajectory.compute(t);
         simpleTrajectory.getLinearData(positionToPackSimple, velocityToPackSimple, accelerationToPackSimple);

         if ((t < (numberOfWaypoints - 2.0) * trajectoryTime / (numberOfWaypoints - 1.0)) && ((t > trajectoryTime / (numberOfWaypoints - 1.0))))
         {
            boolean positionEqual = positionToPackMultiple.epsilonEquals(positionToPackSimple, EPSILON);
            assertTrue(positionEqual);
         }

         if ((t < (numberOfWaypoints - 2.0) * trajectoryTime / (numberOfWaypoints - 1.0)) && ((t > trajectoryTime / (numberOfWaypoints - 1.0))))
         {
            boolean velocityEqual = velocityToPackMultiple.epsilonEquals(velocityToPackSimple, 5.0 * EPSILON);
            assertTrue(velocityEqual);
         }
         else if (t == trajectoryTime)
         {
            boolean velocityEqual = velocityToPackMultiple.epsilonEquals(new Vector3d(0.1, 0.2, 0.3), 5.0 * EPSILON);
            assertTrue(velocityEqual);
         }
         else if (t == 0)
         {
            boolean velocityEqual = velocityToPackMultiple.epsilonEquals(new Vector3d(0.1, 0.2, 0.3), 5.0 * EPSILON);
            assertTrue(velocityEqual);
         }

      }

   }

}
