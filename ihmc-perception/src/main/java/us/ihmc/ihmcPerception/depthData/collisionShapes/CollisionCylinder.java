package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class CollisionCylinder extends CollisionShape
{
   private final double radiusSquared;
   private final double halfLength;

   private Cylinder3D shape3D;

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

   @Override
   public Shape3DReadOnly getOrCreateShape3D()
   {
      if (shape3D == null)
         shape3D = new Cylinder3D(new Point3D(getPose().getTranslation()), new Vector3D(getPose().getM02(), getPose().getM12(), getPose().getM22()), 2.0 * halfLength, Math.sqrt(radiusSquared));
      return shape3D;
   }
}
