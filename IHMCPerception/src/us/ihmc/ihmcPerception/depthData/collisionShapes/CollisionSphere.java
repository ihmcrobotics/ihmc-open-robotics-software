package us.ihmc.ihmcPerception.depthData.collisionShapes;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

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
   public boolean contains(Point3d point)
   {
      return (point.x * point.x + point.y * point.y + point.z * point.z) <= radiusSquared;
   }
}
