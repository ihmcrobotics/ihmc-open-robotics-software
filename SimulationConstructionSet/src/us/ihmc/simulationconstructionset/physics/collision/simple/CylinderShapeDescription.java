package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CylinderShapeDescription<T extends CylinderShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private double height;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   public CylinderShapeDescription(double radius, double height)
   {
      this.radius = radius;
      this.height = height;
   }

   @Override
   public CylinderShapeDescription<T> copy()
   {
      CylinderShapeDescription<T> copy = new CylinderShapeDescription<T>(radius, height);
      copy.transform.set(this.transform);
      return copy;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      transform.multiply(transformToWorld, transform);
   }

   @Override
   public void setFrom(T cylinder)
   {
      this.radius = cylinder.getRadius();
      this.height = cylinder.getHeight();

      cylinder.getTransform(this.transform);
   }

}
