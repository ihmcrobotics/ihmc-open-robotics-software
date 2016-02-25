package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface EuclideanWaypointInterface extends WaypointInterface
{
   public abstract void getPosition(Point3d positionToPack);
   public abstract void getLinearVelocity(Vector3d linearVelocityToPack);
}