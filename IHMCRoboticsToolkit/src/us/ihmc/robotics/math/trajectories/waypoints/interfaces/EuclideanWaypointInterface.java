package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.transformables.Transformable;

public interface EuclideanWaypointInterface<T extends EuclideanWaypointInterface<T>> extends Transformable, WaypointInterface<T>
{
   public abstract void setPosition(Point3d position);

   public abstract void setLinearVelocity(Vector3d linearVelocity);

   public abstract void setPositionToZero();

   public abstract void setLinearVelocityToZero();

   public abstract void setPositionToNaN();

   public abstract void setLinearVelocityToNaN();

   public abstract double positionDistance(T other);

   public abstract void getPosition(Point3d positionToPack);

   public abstract void getLinearVelocity(Vector3d linearVelocityToPack);
}
