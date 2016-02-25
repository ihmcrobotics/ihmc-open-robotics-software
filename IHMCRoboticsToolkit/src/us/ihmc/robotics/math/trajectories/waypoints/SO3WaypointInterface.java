package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public interface SO3WaypointInterface<T extends SO3WaypointInterface<T>> extends WaypointInterface<T>
{
   public abstract void getOrientation(Quat4d orientationToPack);
   public abstract void getAngularVelocity(Vector3d angularVelocityToPack);
}
