package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.YoSE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSE3TrajectoryPoint implements SE3TrajectoryPointBasics
{
   private final YoSE3Waypoint se3Waypoint;
   private final YoDouble time;

   private final String namePrefix;
   private final String nameSuffix;

   public YoSE3TrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      se3Waypoint = new YoSE3Waypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
   }

   @Override
   public EuclideanWaypointBasics getEuclideanWaypoint()
   {
      return se3Waypoint.getEuclideanWaypoint();
   }

   @Override
   public SO3WaypointBasics getSO3Waypoint()
   {
      return se3Waypoint.getSO3Waypoint();
   }

   @Override
   public void setTime(double time)
   {
      this.time.set(time);
   }

   @Override
   public double getTime()
   {
      return time.getValue();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getTime(), getOrientation(), getAngularVelocity());
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
