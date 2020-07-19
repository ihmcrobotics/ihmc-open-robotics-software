package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;


public class MultipleWaypointsOrientationTrajectoryGeneratorTest
{
   private final double EPSILON = 1e-3;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testCompareWithSimple()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      double trajectoryTime = 1.0;
      double dt = 0.0001;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SimpleOrientationTrajectoryGenerator simpleTraj = new SimpleOrientationTrajectoryGenerator("simpleTraj", true, worldFrame, registry);
      simpleTraj.setTrajectoryTime(trajectoryTime);
      simpleTraj.setInitialOrientation(new FrameQuaternion(worldFrame, 1.0, 0.2, -0.5));
      simpleTraj.setFinalOrientation(new FrameQuaternion(worldFrame, -0.3, 0.7, 1.0));
      simpleTraj.initialize();

      int numberOfWaypoints = 100;
      MultipleWaypointsOrientationTrajectoryGenerator multipleWaypointTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("testedTraj", numberOfWaypoints+1, worldFrame, registry);
      multipleWaypointTrajectory.clear();



      FrameQuaternion waypointOrientation = new FrameQuaternion();
      FrameVector3D waypointAngularVelocity = new FrameVector3D();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = numberOfWaypoints == 1 ? trajectoryTime / 2.0 : i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         simpleTraj.getOrientation(waypointOrientation);
         simpleTraj.getAngularVelocity(waypointAngularVelocity);
         multipleWaypointTrajectory.appendWaypoint(timeAtWaypoint, waypointOrientation, waypointAngularVelocity);
      }

      multipleWaypointTrajectory.initialize();


      FrameQuaternion orientationToPackMultiple = new FrameQuaternion(worldFrame);
      FrameVector3D angularVelocityToPackMultiple = new FrameVector3D(worldFrame);
      FrameVector3D angularAccelerationToPackMultiple = new FrameVector3D(worldFrame);

      FrameQuaternion orientationToPackSimple = new FrameQuaternion(worldFrame);
      FrameVector3D angularVelocityToPackSimple = new FrameVector3D(worldFrame);
      FrameVector3D angularAccelerationToPackSimple = new FrameVector3D(worldFrame);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         multipleWaypointTrajectory.compute(t);
         multipleWaypointTrajectory.getAngularData(orientationToPackMultiple, angularVelocityToPackMultiple, angularAccelerationToPackMultiple);

         simpleTraj.compute(t);
         simpleTraj.getAngularData(orientationToPackSimple, angularVelocityToPackSimple, angularAccelerationToPackSimple);

         boolean orientationsEqual = orientationToPackMultiple.epsilonEquals(orientationToPackSimple, EPSILON);
         assertTrue(orientationsEqual);

         boolean angularVelocityEqual =  angularVelocityToPackMultiple.epsilonEquals(angularVelocityToPackSimple, EPSILON);
         assertTrue(angularVelocityEqual);

         boolean angularAccelerationEqual = angularAccelerationToPackMultiple.epsilonEquals(angularAccelerationToPackSimple, EPSILON);
         angularAccelerationToPackMultiple.sub(angularAccelerationToPackSimple);
         //System.out.println("Difference = " + angularAccelerationToPackMultiple.length());
         //assertTrue(angularAccelerationEqual);
      }
   }

}
