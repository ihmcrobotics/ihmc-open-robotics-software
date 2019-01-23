package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;

public class SO3TrajectoryPoint implements SO3TrajectoryPointInterface
{
   private final SO3Waypoint so3Waypoint = new SO3Waypoint();
   private double time;

   public SO3TrajectoryPoint()
   {
   }

   public SO3TrajectoryPoint(SO3TrajectoryPointInterface other)
   {
      set(other);
   }

   public SO3TrajectoryPoint(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      set(time, orientation, angularVelocity);
   }

   @Override
   public QuaternionReadOnly getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      so3Waypoint.setOrientation(x, y, z, s);
   }

   @Override
   public Vector3DReadOnly getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      so3Waypoint.setAngularVelocity(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      so3Waypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      so3Waypoint.applyInverseTransform(transform);
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
   public String toString()
   {
      return "SO3 trajectory point: (time = " + WaypointToStringTools.format(getTime()) + ", " + WaypointToStringTools.waypointToString(so3Waypoint) + ")";
   }
}