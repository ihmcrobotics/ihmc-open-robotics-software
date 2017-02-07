package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class PolytopeShapeDescription<T extends PolytopeShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private final ConvexPolytope polytope;
   private double smoothingRadius = 0.0;

   public PolytopeShapeDescription(ConvexPolytope polytope)
   {
      this.polytope = polytope;
   }

   @Override
   public PolytopeShapeDescription<T> copy()
   {
      ConvexPolytope polytopeCopy = new ConvexPolytope(polytope);
      PolytopeShapeDescription<T> copy = new PolytopeShapeDescription<T>(polytopeCopy);
      copy.setSmoothingRadius(smoothingRadius);
      return copy;
   }

   public ConvexPolytope getPolytope()
   {
      return polytope;
   }

   @Override
   public void setFrom(T polytopeShapeDescription)
   {
      this.polytope.copyVerticesFrom(polytopeShapeDescription.getPolytope());
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      this.polytope.applyTransform(transform);
   }

   public void setSmoothingRadius(double smoothingRadius)
   {
      this.smoothingRadius = smoothingRadius;
   }

   public double getSmoothingRadius()
   {
      return smoothingRadius;
   }

   @Override
   public void getBoundingBox(BoundingBox3d boundingBoxToPack)
   {
      polytope.getBoundingBox(boundingBoxToPack);
   }

}
