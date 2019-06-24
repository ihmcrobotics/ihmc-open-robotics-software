package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CollisionSphere extends CollisionShape
{
   private final double radiusSquared;
   private Shape3DReadOnly shape3D;

   public CollisionSphere(RigidBodyTransform pose, double radius)
   {
      super(pose);
      this.radiusSquared = radius * radius;
   }

   public double getRadius()
   {
      return Math.sqrt(radiusSquared);
   }

   @Override
   public boolean contains(Point3D point)
   {
      return (point.getX() * point.getX() + point.getY() * point.getY() + point.getZ() * point.getZ()) <= radiusSquared;
   }

   @Override
   public Shape3DReadOnly getOrCreateShape3D()
   {
      if (shape3D == null)
         shape3D = new Sphere3D(getPose().getTranslationX(), getPose().getTranslationY(), getPose().getTranslationZ(), getRadius());
      return shape3D;
   }
}
