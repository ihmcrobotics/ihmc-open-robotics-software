package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.robotics.geometry.BoundingBox3d;
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

   private static final GilbertJohnsonKeerthiCollisionDetector detectorForCheckingPointInside = new GilbertJohnsonKeerthiCollisionDetector();
   private static final Point3D tempPointA = new Point3D();
   private static final Point3D tempPointB = new Point3D();
   
   @Override
   public boolean isPointInside(Point3D pointInWorld)
   {      
      //TODO: Reduce garbage generation here.
      SupportingVertexHolder polytopeA = new SupportingVertexHolder()
      {
         @Override
         public Point3D getSupportingVertex(Vector3D supportDirection)
         {
            return pointInWorld;
         }
      };
 
      return detectorForCheckingPointInside.arePolytopesColliding(polytopeA, this.polytope, tempPointA, tempPointB);
   }

   @Override
   public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
   {
      if (smoothingRadius != 0.0)
      {
         throw new RuntimeException("Implement me for nonzero smoothing radius!");
      }
      
      return false;
   }

}
