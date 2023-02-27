package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.SO3Waypoint;

public class SO3TrajectoryPoint implements SO3TrajectoryPointBasics
{
   private final SO3Waypoint so3Waypoint = new SO3Waypoint();
   private double time;

   public SO3TrajectoryPoint()
   {
   }

   public SO3TrajectoryPoint(SO3TrajectoryPointBasics other)
   {
      set(other);
   }

   public SO3TrajectoryPoint(double time, Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      set(time, orientation, angularVelocity);
   }

   @Override
   public Quaternion getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public Vector3D getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
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