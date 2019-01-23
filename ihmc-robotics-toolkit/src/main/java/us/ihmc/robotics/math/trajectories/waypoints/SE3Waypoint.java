package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;

public class SE3Waypoint implements SE3WaypointBasics
{
   private final EuclideanWaypoint euclideanWaypoint = new EuclideanWaypoint();
   private final SO3Waypoint so3Waypoint = new SO3Waypoint();

   @Override
   public void setPosition(double x, double y, double z)
   {
      euclideanWaypoint.setPosition(x, y, z);
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      so3Waypoint.setOrientation(x, y, z, s);
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      euclideanWaypoint.setLinearVelocity(x, y, z);
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      so3Waypoint.setAngularVelocity(x, y, z);
   }

   @Override
   public Point3DReadOnly getPosition()
   {
      return euclideanWaypoint.getPosition();
   }

   @Override
   public QuaternionReadOnly getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public Vector3DReadOnly getLinearVelocity()
   {
      return euclideanWaypoint.getLinearVelocity();
   }

   @Override
   public Vector3DReadOnly getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      euclideanWaypoint.applyTransform(transform);
      so3Waypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      euclideanWaypoint.applyInverseTransform(transform);
      so3Waypoint.applyInverseTransform(transform);
   }

   public EuclideanWaypoint getEuclideanWaypoint()
   {
      return euclideanWaypoint;
   }

   public SO3Waypoint getSO3Waypoint()
   {
      return so3Waypoint;
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
