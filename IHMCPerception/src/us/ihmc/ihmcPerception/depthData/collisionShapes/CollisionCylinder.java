package us.ihmc.ihmcPerception.depthData.collisionShapes;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

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
   public boolean contains(Point3d point)
   {
      return ((point.x * point.x + point.y * point.y) <= radiusSquared) && (point.z >= -halfLength && point.z <= halfLength);

   }
}
