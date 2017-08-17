package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class MultipleWaypointsPositionTrajectoryGeneratorTest
{

 private final double EPSILON = 1e-3;
   
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void test()
   {
      YoVariableRegistry registry = new YoVariableRegistry("traj");

      double trajectoryTime = 1.0;
      double dt = 0.001;
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      MultipleWaypointsPositionTrajectoryGenerator multipleWaypointTrajectory;
      StraightLinePositionTrajectoryGenerator simpleTrajectory;
      
      
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint3D(worldFrame, 1.0, 0.0, 1.0));
      PositionProvider finalPositionProvider = new ConstantPositionProvider(new FramePoint3D(worldFrame,   0.2, 1.0, 0.4));
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
         simpleTrajectory.getPosition(waypointPosition);
         simpleTrajectory.getVelocity(waypointVelocity);
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

}
