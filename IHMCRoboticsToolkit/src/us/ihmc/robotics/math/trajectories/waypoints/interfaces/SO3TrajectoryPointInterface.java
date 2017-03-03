package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;

public interface SO3TrajectoryPointInterface<T extends SO3TrajectoryPointInterface<T>> extends TrajectoryPointInterface<T>, SO3WaypointInterface<T>
{
   public abstract void getOrientation(QuaternionBasics orientationToPack);

   public abstract void getAngularVelocity(Vector3DBasics angularVelocityToPack);
}
