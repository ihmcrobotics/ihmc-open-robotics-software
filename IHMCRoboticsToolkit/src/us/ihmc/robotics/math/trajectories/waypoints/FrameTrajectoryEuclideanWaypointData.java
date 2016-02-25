package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class FrameTrajectoryEuclideanWaypointData extends FrameTrajectoryWaypointData<FrameTrajectoryEuclideanWaypointData, FrameEuclideanWaypoint>
{
   public FrameTrajectoryEuclideanWaypointData()
   {
      super(FrameEuclideanWaypoint.class);
   }

   public void addWaypoint(double time, Point3d position, Vector3d linearVelocity)
   {
      FrameEuclideanWaypoint newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(time, position, linearVelocity);
   }

   public void addWaypoint(EuclideanWaypointInterface<?> waypoint)
   {
      FrameEuclideanWaypoint newWaypoint = addAndInitializeWaypoint();
      newWaypoint.set(waypoint);
   }
}
