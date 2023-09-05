package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;

public class SO3Waypoint implements SO3WaypointBasics
{
   private final Quaternion orientation = new Quaternion();
   private final Vector3D angularVelocity = new Vector3D();

   public SO3Waypoint()
   {
   }

   public SO3Waypoint(Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      set(orientation, angularVelocity);
   }

   @Override
   public Quaternion getOrientation()
   {
      return orientation;
   }

   @Override
   public Vector3D getAngularVelocity()
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
