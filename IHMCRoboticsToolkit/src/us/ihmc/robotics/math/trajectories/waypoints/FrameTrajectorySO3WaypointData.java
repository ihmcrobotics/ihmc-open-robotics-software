package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class FrameTrajectorySO3WaypointData extends FrameTrajectoryWaypointData<FrameTrajectorySO3WaypointData, FrameSO3Waypoint>
{
   public FrameTrajectorySO3WaypointData()
   {
      super(FrameSO3Waypoint.class);
   }

   public void addWaypoint(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      FrameSO3Waypoint newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(time, orientation, angularVelocity);
   }

   public void addWaypoint(SO3WaypointInterface<?> waypoint)
   {
      FrameSO3Waypoint newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(waypoint);
   }
}
