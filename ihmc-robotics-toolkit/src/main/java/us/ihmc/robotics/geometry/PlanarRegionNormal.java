package us.ihmc.robotics.geometry;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

class PlanarRegionNormal implements UnitVector3DReadOnly
{
   private final RigidBodyTransform fromLocalToWorldTransform;

   // no-arg constructor for serialization
   private PlanarRegionNormal()
   {
      this.fromLocalToWorldTransform = null;
   }

   PlanarRegionNormal(RigidBodyTransform fromLocalToWorldTransform)
   {
      this.fromLocalToWorldTransform = fromLocalToWorldTransform;
   }

   @Override
   public double getX()
   {
      return getRawX();
   }

   @Override
   public double getY()
   {
      return getRawY();
   }

   @Override
   public double getZ()
   {
      return getRawZ();
   }

   @Override
   public double getRawX()
   {
      return fromLocalToWorldTransform.getM02();
   }

   @Override
   public double getRawY()
   {
      return fromLocalToWorldTransform.getM12();
   }

   @Override
   public double getRawZ()
   {
      return fromLocalToWorldTransform.getM22();
   }

   @Override
   public boolean isDirty()
   {
      return false;
   }
}
