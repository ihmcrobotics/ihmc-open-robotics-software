package us.ihmc.robotics;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface RegionInWorldInterface<T extends RegionInWorldInterface> extends Plane3DReadOnly
{
   int getRegionId();

   /**
    * Returns the transform from the world frame to the local frame.
    */
   RigidBodyTransformReadOnly getTransformToLocal();

   /**
    * Returns the transform from the local frame to the world frame.
    */
   RigidBodyTransformReadOnly getTransformToWorld();

   /**
    * Returns the origin of the region in the world.
    */
   default void getOrigin(Point3DBasics originToPack)
   {
      originToPack.set(getPoint());
   }

   /**
    * Returns the normal of the region relative to the world.
    */
   default void getNormal(Vector3DBasics normalToPack)
   {
      normalToPack.set(getNormal());
   }

   /**
    * Checks to see if a point defined in the region frame is within the region bounds.
    */
   boolean isPointInside(double xInLocal, double yInLocal);

   void set(T other);

   BoundingBox3DReadOnly getBoundingBox3dInWorld();
}
