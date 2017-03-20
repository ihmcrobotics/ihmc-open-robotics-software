package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SO3WaypointInterface<T extends SO3WaypointInterface<T>> extends GeometryObject<T>
{
   public abstract void setOrientation(QuaternionReadOnly orientation);

   public abstract void setAngularVelocity(Vector3DReadOnly angularVelocity);

   public abstract void setOrientationToZero();

   public abstract void setAngularVelocityToZero();

   public abstract void setOrientationToNaN();

   public abstract void setAngularVelocityToNaN();

   public abstract void getOrientation(QuaternionBasics orientationToPack);

   public abstract void getAngularVelocity(Vector3DBasics angularVelocityToPack);
}
