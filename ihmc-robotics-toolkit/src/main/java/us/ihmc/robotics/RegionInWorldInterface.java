package us.ihmc.robotics;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface RegionInWorldInterface<T extends RegionInWorldInterface>
{
   RigidBodyTransformReadOnly getTransformToLocal();
   RigidBodyTransformReadOnly getTransformToWorld();

   void getOrigin(Point3DBasics originToPack);
   void getNormal(Vector3DBasics normalToPack);

   boolean isPointInside(double xInLocal, double yInLocal);

   void set(T other);

   BoundingBox3DReadOnly getBoundingBox3dInWorld();
}
