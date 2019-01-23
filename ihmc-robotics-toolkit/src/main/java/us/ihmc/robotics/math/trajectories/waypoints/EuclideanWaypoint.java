package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;

public class EuclideanWaypoint implements EuclideanWaypointBasics
{
   private final Point3D position = new Point3D();
   private final Vector3D linearVelocity = new Vector3D();

   @Override
   public Point3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public Vector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      linearVelocity.set(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      position.applyTransform(transform);
      linearVelocity.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      position.applyInverseTransform(transform);
      linearVelocity.applyInverseTransform(transform);
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
