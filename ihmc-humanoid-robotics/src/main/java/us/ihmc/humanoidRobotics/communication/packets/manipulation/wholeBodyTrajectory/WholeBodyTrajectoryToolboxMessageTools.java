package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.robotics.screwTheory.RigidBody;

public class WholeBodyTrajectoryToolboxMessageTools
{
   public static interface FunctionTrajectory
   {
      public Pose3D compute(double time);
   }

   public static WaypointBasedTrajectoryMessage createTrajectoryMessage(RigidBody endEffector, double t0, double tf, double timeResolution,
                                                                        FunctionTrajectory trajectoryToDiscretize,
                                                                        ConfigurationSpaceName... unconstrainedDegreesOfFreedom)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      // Adjust the timeResolution using the numberOfWaypoints:
      timeResolution = (tf - t0) / (numberOfWaypoints - 1);

      double[] waypointTimes = new double[numberOfWaypoints];
      Pose3D[] waypoints = new Pose3D[numberOfWaypoints];

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double waypointTime = i * timeResolution + t0;

         waypointTimes[i] = waypointTime;
         waypoints[i] = trajectoryToDiscretize.compute(waypointTime);
      }

      return new WaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints, unconstrainedDegreesOfFreedom);
   }
}
