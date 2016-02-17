package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface EuclideanWaypointInterface
{
   public abstract double getTime();
   public abstract void getPosition(Point3d positionToPack);
   public abstract void getLinearVelocity(Vector3d linearVelocityToPack);
}