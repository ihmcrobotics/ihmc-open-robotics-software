package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSO3Waypoint implements SO3WaypointBasics
{
   private final YoQuaternion orientation;
   private final YoVector3D angularVelocity;

   public YoSO3Waypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      orientation = new YoQuaternion(namePrefix + "Orientation", nameSuffix, registry);
      angularVelocity = new YoVector3D(namePrefix + "AngularVelocity", nameSuffix, registry);
   }

   @Override
   public QuaternionBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public Vector3DBasics getAngularVelocity()
   {
      return angularVelocity;
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
      else if (object instanceof SO3WaypointReadOnly)
         return equals((SO3WaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
