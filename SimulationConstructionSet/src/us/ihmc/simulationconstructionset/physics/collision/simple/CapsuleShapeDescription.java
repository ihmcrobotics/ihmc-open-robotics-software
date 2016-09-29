package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CapsuleShapeDescription<T extends CapsuleShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private LineSegment3d lineSegment = new LineSegment3d();

   public CapsuleShapeDescription(double radius, LineSegment3d lineSegment)
   {
      this.radius = radius;
      this.lineSegment.set(lineSegment);
   }

   @Override
   public CapsuleShapeDescription<T> copy()
   {
      CapsuleShapeDescription<T> copy = new CapsuleShapeDescription<T>(radius, lineSegment);
      return copy;
   }

   public double getRadius()
   {
      return radius;
   }

   public void getLineSegment(LineSegment3d lineSegmentToPack)
   {
      lineSegmentToPack.set(lineSegment);
   }

   @Override
   public void setFrom(T sphereShapeDescription)
   {
      this.radius = sphereShapeDescription.getRadius();
      sphereShapeDescription.getLineSegment(this.lineSegment);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      lineSegment.applyTransform(transform);
   }

}
