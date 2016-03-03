package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;

public interface SO3TrajectoryPointInterface<T extends SO3TrajectoryPointInterface<T>> extends TrajectoryPointInterface<T>, SO3WaypointInterface<T>
{
   public abstract void getOrientation(Quat4d orientationToPack);

   public abstract void getAngularVelocity(Vector3d angularVelocityToPack);
}
