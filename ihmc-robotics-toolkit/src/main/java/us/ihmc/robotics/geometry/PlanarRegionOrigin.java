package us.ihmc.robotics.geometry;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class PlanarRegionOrigin implements Point3DReadOnly
{
   private final RigidBodyTransform fromLocalToWorldTransform;

   // no-arg constructor for serialization
   private PlanarRegionOrigin()
   {
      this.fromLocalToWorldTransform = null;
   }

   public PlanarRegionOrigin(RigidBodyTransform fromLocalToWorldTransform)
   {
      this.fromLocalToWorldTransform = fromLocalToWorldTransform;
   }

   @Override
   public double getX()
   {
      return fromLocalToWorldTransform.getM03();
   }

   @Override
   public double getY()
   {
      return fromLocalToWorldTransform.getM13();
   }

   @Override
   public double getZ()
   {
      return fromLocalToWorldTransform.getM23();
   }
}
