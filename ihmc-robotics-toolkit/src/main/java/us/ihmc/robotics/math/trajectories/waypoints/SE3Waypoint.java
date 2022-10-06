package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;

public class SE3Waypoint implements SE3WaypointBasics
{
   private final EuclideanWaypoint euclideanWaypoint = new EuclideanWaypoint();
   private final SO3Waypoint so3Waypoint = new SO3Waypoint();

   public SE3Waypoint()
   {
   }

   public SE3Waypoint(Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      set(position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   public EuclideanWaypoint getEuclideanWaypoint()
   {
      return euclideanWaypoint;
   }

   @Override
   public SO3Waypoint getSO3Waypoint()
   {
      return so3Waypoint;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getEuclideanWaypoint(), getSO3Waypoint());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof SE3WaypointReadOnly)
         return equals((SE3WaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
