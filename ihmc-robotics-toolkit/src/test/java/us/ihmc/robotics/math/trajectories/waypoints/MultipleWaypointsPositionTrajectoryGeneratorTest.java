package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.robotics.math.trajectories.BlendedPositionTrajectoryGeneratorTest;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class MultipleWaypointsPositionTrajectoryGeneratorTest
{

 private final double EPSILON = 1e-3;

 @AfterEach
 public void tearDown()
 {
    ReferenceFrameTools.clearWorldFrameTree();
 }

   
   @Test
   public void test()
   {
      YoRegistry registry = new YoRegistry("traj");

      double trajectoryTime = 1.0;
      double dt = 0.001;
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      MultipleWaypointsPositionTrajectoryGenerator multipleWaypointTrajectory;
      StraightLinePositionTrajectoryGenerator simpleTrajectory;
      
      
      DoubleProvider trajectoryTimeProvider = () -> trajectoryTime;
      FramePositionProvider initialPositionProvider = () -> new FramePoint3D(worldFrame, 1.0, 0.0, 1.0);
      FramePositionProvider finalPositionProvider = () -> new FramePoint3D(worldFrame, 0.2, 1.0, 0.4);
      simpleTrajectory = new StraightLinePositionTrajectoryGenerator("simpleTraj", worldFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, registry);
      simpleTrajectory.initialize();

      int numberOfWaypoints = 11;
      multipleWaypointTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("testedTraj", 50, worldFrame, registry);
      multipleWaypointTrajectory.clear();
      
      

      FramePoint3D waypointPosition = new FramePoint3D();
      FrameVector3D waypointVelocity = new FrameVector3D();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         waypointPosition.setIncludingFrame(simpleTrajectory.getPosition());
         waypointVelocity.setIncludingFrame(simpleTrajectory.getVelocity());
         multipleWaypointTrajectory.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);
      }
      multipleWaypointTrajectory.initialize();

      
      FramePoint3D positionToPackMultiple = new FramePoint3D(worldFrame);
      FrameVector3D velocityToPackMultiple = new FrameVector3D(worldFrame);
      FrameVector3D accelerationToPackMultiple = new FrameVector3D(worldFrame);
      
      FramePoint3D positionToPackSimple = new FramePoint3D(worldFrame);
      FrameVector3D velocityToPackSimple = new FrameVector3D(worldFrame);
      FrameVector3D accelerationToPackSimple = new FrameVector3D(worldFrame);
      
      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         multipleWaypointTrajectory.compute(t);
         multipleWaypointTrajectory.getLinearData(positionToPackMultiple, velocityToPackMultiple, accelerationToPackMultiple);
         
         simpleTrajectory.compute(t);
         simpleTrajectory.getLinearData(positionToPackSimple, velocityToPackSimple, accelerationToPackSimple);
    
         boolean positionEqual = positionToPackMultiple.epsilonEquals(positionToPackSimple, EPSILON);
         assertTrue(positionEqual);
         
         boolean velocityEqual = velocityToPackMultiple.epsilonEquals(velocityToPackSimple, 5.0*EPSILON);
         assertTrue(velocityEqual);

         // The straight line trajectory does minimum jerk whereas the multiple waypoint uses cubic splines
//         boolean accelerationEqual = accelerationToPackMultiple.epsilonEquals(accelerationToPackSimple, 100.0*EPSILON);
//         assertTrue(accelerationEqual);
      }

   }

   @Test
   public void testRandomCreation()
   {
      Random random = new Random(1738L);
      double trajectoryDuration = 1.0;
      int numberOfSamples = 10;

      List<FrameEuclideanTrajectoryPoint> waypoints = createRandomWaypoints(random, numberOfSamples, trajectoryDuration, ReferenceFrame.getWorldFrame());
      MultipleWaypointsPositionTrajectoryGenerator trajectory = createRandomReferenceTrajectory(waypoints, ReferenceFrame.getWorldFrame(), new YoRegistry("Test"));

      FrameEuclideanTrajectoryPoint firstWaypoint = waypoints.get(0);
      FrameEuclideanTrajectoryPoint lastWaypoint = waypoints.get(waypoints.size() - 1);

      assertEquals(0.0, firstWaypoint.getTime(), EPSILON);
      assertEquals(trajectoryDuration, lastWaypoint.getTime(), EPSILON);
      assertEquals(lastWaypoint.getTime(), trajectory.getLastWaypointTime(), EPSILON);

      FramePoint3D startPosition = new FramePoint3D(firstWaypoint.getPosition());
      FramePoint3D lastPosition = new FramePoint3D(lastWaypoint.getPosition());

      double dt = 0.005;
      int itersPerTick = 5;
      for (double time = -1.0; time < 0.0; time += dt)
      {
         for (int iter = 0; iter < itersPerTick; iter++)
         {
            trajectory.compute(time);
            EuclidFrameTestTools.assertGeometricallyEquals(startPosition, trajectory.getPosition(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(new FrameVector3D(), trajectory.getVelocity(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(new FrameVector3D(), trajectory.getAcceleration(), EPSILON);
         }
      }

      for (double time = 0.0; time <= trajectory.getLastWaypointTime(); time += dt)
      {
         int firstWaypointIdx = getFirstWaypoint(time, waypoints);

         FrameEuclideanTrajectoryPoint startWaypoint = waypoints.get(firstWaypointIdx);
         FrameEuclideanTrajectoryPoint endWaypoint = waypoints.get(firstWaypointIdx + 1);
         double duration = endWaypoint.getTime() - startWaypoint.getTime();

         FramePolynomial3D polynomial = new FramePolynomial3D(5, ReferenceFrame.getWorldFrame());
         polynomial.setCubic(0.0,
                             duration,
                             startWaypoint.getPosition(),
                             startWaypoint.getLinearVelocity(),
                             endWaypoint.getPosition(),
                             endWaypoint.getLinearVelocity());
         polynomial.compute(time - startWaypoint.getTime());

         for (int iter = 0; iter < itersPerTick; iter++)
         {
            trajectory.compute(time);
            assertEquals(firstWaypointIdx, trajectory.getCurrentWaypointIndex());
            EuclidFrameTestTools.assertGeometricallyEquals(polynomial.getPosition(), trajectory.getPosition(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(polynomial.getVelocity(), trajectory.getVelocity(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(polynomial.getAcceleration(), trajectory.getAcceleration(), EPSILON);
         }
      }

      for (double time = trajectory.getLastWaypointTime(); time >= 0.0; time -= dt)
      {
         int firstWaypointIdx = getFirstWaypoint(time, waypoints);

         FrameEuclideanTrajectoryPoint startWaypoint = waypoints.get(firstWaypointIdx);
         FrameEuclideanTrajectoryPoint endWaypoint = waypoints.get(firstWaypointIdx + 1);
         double duration = endWaypoint.getTime() - startWaypoint.getTime();

         FramePolynomial3D polynomial = new FramePolynomial3D(5, ReferenceFrame.getWorldFrame());
         polynomial.setCubic(0.0,
                             duration,
                             startWaypoint.getPosition(),
                             startWaypoint.getLinearVelocity(),
                             endWaypoint.getPosition(),
                             endWaypoint.getLinearVelocity());
         polynomial.compute(time - startWaypoint.getTime());

         for (int iter = 0; iter < itersPerTick; iter++)
         {
            trajectory.compute(time);
            assertEquals(firstWaypointIdx, trajectory.getCurrentWaypointIndex());
            EuclidFrameTestTools.assertGeometricallyEquals(polynomial.getPosition(), trajectory.getPosition(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(polynomial.getVelocity(), trajectory.getVelocity(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(polynomial.getAcceleration(), trajectory.getAcceleration(), EPSILON);
         }
      }

      for (double time = trajectoryDuration + dt; time <= trajectoryDuration + 1.0; time += dt)
      {
         for (int iter = 0; iter < itersPerTick; iter++)
         {
            trajectory.compute(time);
            EuclidFrameTestTools.assertGeometricallyEquals(lastPosition, trajectory.getPosition(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(new FrameVector3D(), trajectory.getVelocity(), EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(new FrameVector3D(), trajectory.getAcceleration(), EPSILON);
         }
      }
   }

   private int getFirstWaypoint(double time, List<FrameEuclideanTrajectoryPoint> waypoints)
   {
      for (int i = 0; i < waypoints.size() - 1; i++)
      {
         if (time < waypoints.get(i).getTime())
            return -1;
         if (time >= waypoints.get(i).getTime() && time <= waypoints.get(i + 1).getTime())
            return i;
      }

      return -1;
   }

   private List<FrameEuclideanTrajectoryPoint> createRandomWaypoints(Random random, int numberOfWaypoints, double duration, ReferenceFrame referenceFrame)
   {
      List<FrameEuclideanTrajectoryPoint> waypoints = new ArrayList<>();
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double time = i * duration / (numberOfWaypoints - 1);
         PositionTrajectoryState state = new PositionTrajectoryState(random, time, referenceFrame);
         waypoints.add(state.getWaypoint());
      }
      return waypoints;
   }

   private MultipleWaypointsPositionTrajectoryGenerator createRandomReferenceTrajectory(List<FrameEuclideanTrajectoryPoint> waypoints,
                                                                                         ReferenceFrame referenceFrame, YoRegistry registry)
   {
      MultipleWaypointsPositionTrajectoryGenerator referenceTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("referenceTrajectory", 10, referenceFrame, registry);
      for (int i = 0; i < waypoints.size(); i++)
      {
         referenceTrajectory.appendWaypoint(waypoints.get(i));
      }
      referenceTrajectory.initialize();
      return referenceTrajectory;
   }

   private class PositionTrajectoryState
   {
      public final double time;
      public final FramePoint3D position;
      public final FrameVector3D linearVelocity;
      public final FrameVector3D linearAcceleration;
      private final ReferenceFrame expressedInFrame;

      public PositionTrajectoryState(FixedFramePositionTrajectoryGenerator trajectory, double time,  ReferenceFrame expressedInFrame)
      {
         this.position = new FramePoint3D(expressedInFrame);
         this.linearVelocity = new FrameVector3D(expressedInFrame);
         this.linearAcceleration = new FrameVector3D(expressedInFrame);
         this.time = time;
         this.expressedInFrame = expressedInFrame;
         trajectory.compute(time);
         trajectory.getLinearData(position, linearVelocity, linearAcceleration);
      }

      public PositionTrajectoryState(Random random, double time, ReferenceFrame expressedInFrame)
      {
         this.time = time;
         this.position = EuclidFrameRandomTools.nextFramePoint3D(random, expressedInFrame, 1.0, 1.0, 1.0);
         this.linearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         this.linearAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, expressedInFrame, -100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
         this.expressedInFrame = expressedInFrame;
      }

      public FrameEuclideanTrajectoryPoint getWaypoint()
      {
         return new FrameEuclideanTrajectoryPoint(time, position, linearVelocity);
      }

      public FramePoint3DReadOnly getPosition()
      {
         position.changeFrame(expressedInFrame);
         return new FramePoint3D(position);
      }

      public FrameVector3DReadOnly getLinearVelocity()
      {
         linearVelocity.changeFrame(expressedInFrame);
         return new FrameVector3D(linearVelocity);
      }

      public void assertEpsilonEquals(PositionTrajectoryState other, double epsilon)
      {
         EuclidFrameTestTools.assertGeometricallyEquals(position, other.position, epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(linearVelocity, other.linearVelocity, epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(linearAcceleration, other.linearAcceleration, epsilon);
      }
   }

}
