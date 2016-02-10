package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface EuclideanWaypointInterface
{
   public abstract double getTime();
   public abstract Point3d getPosition();
   public abstract Vector3d getLinearVelocity();
}