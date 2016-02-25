package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class FrameTrajectorySE3WaypointData extends FrameTrajectoryWaypointData<FrameTrajectorySE3WaypointData, FrameSE3Waypoint>
{
   public FrameTrajectorySE3WaypointData()
   {
      super(FrameSE3Waypoint.class);
   }

   public void addWaypoint(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      FrameSE3Waypoint newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void addWaypoint(SE3WaypointInterface<?> waypoint)
   {
      FrameSE3Waypoint newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(waypoint);
   }
}
