package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public interface SO3WaypointInterface<T extends SO3WaypointInterface<T>> extends TransformableGeometryObjectInterface, WaypointInterface<T>
{
   public abstract void setOrientation(Quat4d orientation);

   public abstract void setAngularVelocity(Vector3d angularVelocity);

   public abstract void setOrientationToZero();

   public abstract void setAngularVelocityToZero();

   public abstract void setOrientationToNaN();

   public abstract void setAngularVelocityToNaN();

   public abstract void getOrientation(Quat4d orientationToPack);

   public abstract void getAngularVelocity(Vector3d angularVelocityToPack);
}
