package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface EuclideanWaypointInterface<T extends EuclideanWaypointInterface<T>> extends WaypointInterface<T>
{
   public abstract void getPosition(Point3d positionToPack);
   public abstract void getLinearVelocity(Vector3d linearVelocityToPack);
}