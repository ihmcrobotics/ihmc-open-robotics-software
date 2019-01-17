package us.ihmc.robotics.geometry.transformables;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;

public class SO3Waypoint implements SO3WaypointInterface<SO3Waypoint>
{
   private final Quaternion orientation = new Quaternion();
   private final Vector3D angularVelocity = new Vector3D();

   @Override
   public QuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   @Override
   public Vector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      orientation.set(x, y, z, s);
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      angularVelocity.set(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      orientation.applyTransform(transform);
      angularVelocity.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.applyInverseTransform(transform);
      angularVelocity.applyInverseTransform(transform);
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
