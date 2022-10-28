package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.YoSO3Waypoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSO3TrajectoryPoint implements SO3TrajectoryPointBasics
{
   private final YoSO3Waypoint so3Waypoint;
   private final YoDouble time;

   private final String namePrefix;
   private final String nameSuffix;

   public YoSO3TrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      so3Waypoint = new YoSO3Waypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
   }

   @Override
   public QuaternionBasics getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public Vector3DBasics getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
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
      else if (object instanceof SO3TrajectoryPointReadOnly)
         return equals((SO3TrajectoryPointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
