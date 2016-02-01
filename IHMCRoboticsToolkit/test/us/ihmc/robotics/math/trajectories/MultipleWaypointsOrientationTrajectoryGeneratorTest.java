package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;


@DeployableTestClass(targets = {TestPlanTarget.Fast})
public class MultipleWaypointsOrientationTrajectoryGeneratorTest
{
   private final double EPSILON = 1e-3;
   
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testCompareWithSimple()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      
      double trajectoryTime = 1.0;
      double dt = 0.0001;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
    
      SimpleOrientationTrajectoryGenerator simpleTraj = new SimpleOrientationTrajectoryGenerator("simpleTraj", true, worldFrame, registry);
      simpleTraj.setTrajectoryTime(trajectoryTime);
      simpleTraj.setInitialOrientation(new FrameOrientation(worldFrame, 1.0, 0.2, -0.5));
      simpleTraj.setFinalOrientation(new FrameOrientation(worldFrame, -0.3, 0.7, 1.0));
      simpleTraj.initialize();

      int numberOfWaypoints = 100;
      MultipleWaypointsOrientationTrajectoryGenerator multipleWaypointTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("testedTraj", numberOfWaypoints+1, true, true, worldFrame, registry);
      multipleWaypointTrajectory.clear();
      
      
      
      FrameOrientation waypointOrientation = new FrameOrientation();
      FrameVector waypointAngularVelocity = new FrameVector();
      
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = numberOfWaypoints == 1 ? trajectoryTime / 2.0 : i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         simpleTraj.get(waypointOrientation);
         simpleTraj.packAngularVelocity(waypointAngularVelocity);
         multipleWaypointTrajectory.appendWaypoint(timeAtWaypoint, waypointOrientation, waypointAngularVelocity);
      }
      
      multipleWaypointTrajectory.initialize();


      FrameOrientation orientationToPackMultiple = new FrameOrientation(worldFrame);
      FrameVector angularVelocityToPackMultiple = new FrameVector(worldFrame);
      FrameVector angularAccelerationToPackMultiple = new FrameVector(worldFrame);

      FrameOrientation orientationToPackSimple = new FrameOrientation(worldFrame);
      FrameVector angularVelocityToPackSimple = new FrameVector(worldFrame);
      FrameVector angularAccelerationToPackSimple = new FrameVector(worldFrame);
      
      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         multipleWaypointTrajectory.compute(t);
         multipleWaypointTrajectory.packAngularData(orientationToPackMultiple, angularVelocityToPackMultiple, angularAccelerationToPackMultiple);
         
         simpleTraj.compute(t);
         simpleTraj.packAngularData(orientationToPackSimple, angularVelocityToPackSimple, angularAccelerationToPackSimple);
         
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
