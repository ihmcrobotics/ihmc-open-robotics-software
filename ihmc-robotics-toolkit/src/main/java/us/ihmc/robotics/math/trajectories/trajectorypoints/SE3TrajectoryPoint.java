package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.SE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.SO3Waypoint;

public class SE3TrajectoryPoint implements SE3TrajectoryPointBasics
{
   private double time;
   private final SE3Waypoint se3Waypoint = new SE3Waypoint();

   public SE3TrajectoryPoint()
   {
   }

   public SE3TrajectoryPoint(SE3TrajectoryPointBasics other)
   {
      set(other);
   }

   public SE3TrajectoryPoint(double time,
                             Point3DReadOnly position,
                             Orientation3DReadOnly orientation,
                             Vector3DReadOnly linearVelocity,
                             Vector3DReadOnly angularVelocity)
   {
      set(time, position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   public EuclideanWaypoint getEuclideanWaypoint()
   {
      return se3Waypoint.getEuclideanWaypoint();
   }

   @Override
   public SO3Waypoint getSO3Waypoint()
   {
      return se3Waypoint.getSO3Waypoint();
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getTime(), getEuclideanWaypoint(), getSO3Waypoint());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof SE3TrajectoryPointReadOnly)
         return equals((SE3TrajectoryPointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
