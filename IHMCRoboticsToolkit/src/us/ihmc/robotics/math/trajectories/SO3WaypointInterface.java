package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public interface SO3WaypointInterface
{
   public abstract double getTime();
   public abstract void getOrientation(Quat4d orientationToPack);
   public abstract void getAngularVelocity(Vector3d angularVelocityToPack);
}
