package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, 1.0, 0.0, 1.0));
      PositionProvider finalPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame,   0.2, 1.0, 0.4));
      simpleTrajectory = new StraightLinePositionTrajectoryGenerator("simpleTraj", worldFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, registry);
      simpleTrajectory.initialize();

      int numberOfWaypoints = 11;
      multipleWaypointTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("testedTraj", 50, worldFrame, registry);
      multipleWaypointTrajectory.clear();
      
      

      FramePoint waypointPosition = new FramePoint();
      FrameVector waypointVelocity = new FrameVector();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTrajectory.compute(timeAtWaypoint);
         simpleTrajectory.getPosition(waypointPosition);
         simpleTrajectory.getVelocity(waypointVelocity);
         multipleWaypointTrajectory.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);
      }
      multipleWaypointTrajectory.initialize();

      
      FramePoint positionToPackMultiple = new FramePoint(worldFrame);
      FrameVector velocityToPackMultiple = new FrameVector(worldFrame);
      FrameVector accelerationToPackMultiple = new FrameVector(worldFrame);
      
      FramePoint positionToPackSimple = new FramePoint(worldFrame);
      FrameVector velocityToPackSimple = new FrameVector(worldFrame);
      FrameVector accelerationToPackSimple = new FrameVector(worldFrame);
      
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
