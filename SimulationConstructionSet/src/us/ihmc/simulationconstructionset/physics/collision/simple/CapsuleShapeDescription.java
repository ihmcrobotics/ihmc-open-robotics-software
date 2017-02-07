package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CapsuleShapeDescription<T extends CapsuleShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private LineSegment3d lineSegment = new LineSegment3d();
   private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

   public CapsuleShapeDescription(double radius, LineSegment3d lineSegment)
   {
      this.radius = radius;
      this.lineSegment.set(lineSegment);
   }

   public CapsuleShapeDescription(double radius, double height)
   {
      if (height < 2.0 * radius) throw new RuntimeException("Capsule height must be at least 2.0 * radius!");
      this.radius = radius;
      this.lineSegment.set(0.0, 0.0, -height/2.0 + radius, 0.0, 0.0, height/2.0 - radius);
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
   public void setFrom(T capsuleShapeDescription)
   {
      this.radius = capsuleShapeDescription.getRadius();
      capsuleShapeDescription.getLineSegment(this.lineSegment);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      lineSegment.applyTransform(transform);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
