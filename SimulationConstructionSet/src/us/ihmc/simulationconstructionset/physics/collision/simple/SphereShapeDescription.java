package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class SphereShapeDescription<T extends SphereShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private Point3d center = new Point3d();

   public SphereShapeDescription(double radius, Point3d center)
   {
      this.radius = radius;
      this.center.set(center);
   }

   @Override
   public SphereShapeDescription<T> copy()
   {
      SphereShapeDescription<T> copy = new SphereShapeDescription<T>(radius, center);
      return copy;
   }

   public double getRadius()
   {
      return radius;
   }

   public void getCenter(Point3d centerToPack)
   {
      centerToPack.set(center);
   }

   @Override
   public void setFrom(T sphereShapeDescription)
   {
      this.radius = sphereShapeDescription.getRadius();
      sphereShapeDescription.getCenter(this.center);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(center);
   }

}
