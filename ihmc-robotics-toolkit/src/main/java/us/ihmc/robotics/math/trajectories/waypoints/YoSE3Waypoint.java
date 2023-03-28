package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSE3Waypoint implements SE3WaypointBasics
{
   private final YoEuclideanWaypoint euclidWaypoint;
   private final YoSE3Waypoint so3Waypoint;

   public YoSE3Waypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      euclidWaypoint = new YoEuclideanWaypoint(namePrefix, nameSuffix, registry);
      so3Waypoint = new YoSE3Waypoint(namePrefix, nameSuffix, registry);
   }

   @Override
   public EuclideanWaypointBasics getEuclideanWaypoint()
   {
      return euclidWaypoint;
   }

   @Override
   public SO3WaypointBasics getSO3Waypoint()
   {
      return so3Waypoint;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getOrientation(), getAngularVelocity());
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
