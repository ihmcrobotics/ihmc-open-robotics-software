package us.ihmc.robotics.geometry.interfaces;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface EuclideanWaypointInterface<T extends EuclideanWaypointInterface<T>> extends GeometryObject<T>
{
   public abstract void getPosition(Point3d positionToPack);

   public abstract void setPosition(Point3d position);

   public abstract void setPositionToZero();

   public abstract void setPositionToNaN();

   public abstract double positionDistance(T other);

   public abstract void getLinearVelocity(Vector3d linearVelocityToPack);

   public abstract void setLinearVelocity(Vector3d linearVelocity);

   public abstract void setLinearVelocityToZero();

   public abstract void setLinearVelocityToNaN();

}
