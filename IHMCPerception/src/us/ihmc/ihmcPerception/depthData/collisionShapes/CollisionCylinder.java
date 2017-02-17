package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CollisionCylinder extends CollisionShape
{
   private final double radiusSquared;
   private final double halfLength;

   public CollisionCylinder(RigidBodyTransform pose, double radius, double length)
   {
      super(pose);
      this.radiusSquared = radius * radius;
      this.halfLength = length / 2.0;
   }

   public double getRadius()
   {
      return Math.sqrt(radiusSquared);
   }

   public double getLength()
   {
      return halfLength * 2.0;
   }

   @Override
   public boolean contains(Point3D point)
   {
      return ((point.getX() * point.getX() + point.getY() * point.getY()) <= radiusSquared) && (point.getZ() >= -halfLength && point.getZ() <= halfLength);

   }
}
