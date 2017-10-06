package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CollisionSphere extends CollisionShape
{
   private final double radiusSquared;

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
}
