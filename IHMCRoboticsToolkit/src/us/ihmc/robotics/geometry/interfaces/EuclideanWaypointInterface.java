package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface EuclideanWaypointInterface<T extends EuclideanWaypointInterface<T>> extends GeometryObject<T>
{
   public abstract void getPosition(Point3DBasics positionToPack);

   public abstract void setPosition(Point3DReadOnly position);

   public abstract void setPositionToZero();

   public abstract void setPositionToNaN();

   public abstract double positionDistance(T other);

   public abstract void getLinearVelocity(Vector3DBasics linearVelocityToPack);

   public abstract void setLinearVelocity(Vector3DReadOnly linearVelocity);

   public abstract void setLinearVelocityToZero();

   public abstract void setLinearVelocityToNaN();

}
