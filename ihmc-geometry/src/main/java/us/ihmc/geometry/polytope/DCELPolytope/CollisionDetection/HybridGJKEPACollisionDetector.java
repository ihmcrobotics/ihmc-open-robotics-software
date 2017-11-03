package us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedSimplexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeReadOnly;

public class HybridGJKEPACollisionDetector
{
   private static final Point3D origin = new Point3D();
   private double epsilon = Epsilons.ONE_TEN_THOUSANDTH;

   private final ExtendedSimplexPolytope simplex = new ExtendedSimplexPolytope();
   private Vector3D supportVectorDirectionNegative = new Vector3D();
   private Vector3D supportVectorDirection = new Vector3D()
   {
      @Override
      public final void setX(double x)
      {
         super.setX(x);
         supportVectorDirectionNegative.setX(-x);
      };
      @Override
      public final void setY(double y)
      {
         super.setY(y);
         supportVectorDirectionNegative.setY(-y);
      };
      @Override
      public final void setZ(double x)
      {
         super.setZ(x);
         supportVectorDirectionNegative.setZ(-x);
      };
   };
   private Vector3D previousSupportVectorDirection = new Vector3D();

   private final int iterations = 10;

   public void setSupportVectorDirection(Vector3DReadOnly vectorToSet)
   {
      supportVectorDirection.set(vectorToSet);
   }
   
   public void getSupportVectorDirection(Vector3D vectorToPack)
   {
      vectorToPack.set(supportVectorDirection);
   }

   public void getSupportVectorDirectionNegative(Vector3D vectorToPack)
   {
      vectorToPack.set(supportVectorDirectionNegative);
   }
   
   public HybridGJKEPACollisionDetector()
   {
      this(Epsilons.ONE_BILLIONTH);
   }

   public HybridGJKEPACollisionDetector(double epsilon)
   {
      this.epsilon = epsilon;
   }
   
   public boolean checkCollisionBetweenTwoPolytopes(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, Vector3D initialDirectionForSearch)
   {
      simplex.clear();
      if(polytopeA.isEmpty() || polytopeB.isEmpty())
      {
         return false;
      }
         
      setSupportVectorDirection(initialDirectionForSearch);
      simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
      simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
      previousSupportVectorDirection.set(supportVectorDirection);
      for (int i = 0; i < iterations;)
      {
         simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
         if(simplex.isInteriorPoint(origin, epsilon))
         {
            return true;
         }
         else
            simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
         if(previousSupportVectorDirection.epsilonEquals(supportVectorDirection, epsilon))
            return false;
         else
            previousSupportVectorDirection.set(supportVectorDirection);
      }
      return false;
   }
   
   public void runEPAExpansion(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, Vector3D collisionVectorToPack)
   {
      runEPAExpansion(polytopeA, polytopeB, simplex, supportVectorDirection, collisionVectorToPack);
   }

   public void runEPAExpansion(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, ExtendedSimplexPolytope simplex, Vector3D initialSupportVectorDirection, Vector3D collisionVectorToPack)
   {
      runEPAExpansion(polytopeA, polytopeB, simplex, initialSupportVectorDirection);
      getCollisionVector(simplex, collisionVectorToPack);
   }

   public void getCollisionVector(Vector3D collisionVectorToPack)
   {
      getCollisionVector(simplex, collisionVectorToPack);
   }
   
   private void getCollisionVector(ExtendedSimplexPolytope simplex, Vector3D collisionVectorToPack)
   {
      collisionVectorToPack.set(supportVectorDirection);
      collisionVectorToPack.normalize();
      collisionVectorToPack.scale(simplex.getSmallestSimplexMemberReference(origin).getShortestDistanceTo(origin));
   }

   private void runEPAExpansion(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, ExtendedSimplexPolytope simplex,
                                Vector3D initialSupportVectorDirection)
   {
      supportVectorDirection.set(initialSupportVectorDirection);
      previousSupportVectorDirection.set(initialSupportVectorDirection);
      while(true)
      {
         simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
         simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
         if(supportVectorDirection.epsilonEquals(previousSupportVectorDirection, epsilon))
            break;
         else
            previousSupportVectorDirection.set(supportVectorDirection);
      }
   }
   
   public void runEPAExpansion(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      runEPAExpansion(polytopeA, polytopeB, simplex, supportVectorDirection, pointOnAToPack, pointOnBToPack);
   }
   
   public void runEPAExpansion(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, ExtendedSimplexPolytope simplex, Vector3D initialSupportVectorDirection, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      runEPAExpansion(polytopeA, polytopeB, simplex, initialSupportVectorDirection);
      getCollisionPoints(polytopeA, polytopeB, pointOnAToPack, pointOnBToPack);
   }

   public void getCollisionPoints(ConvexPolytopeReadOnly polytopeA, ConvexPolytopeReadOnly polytopeB, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      simplex.getCollidingPointsOnSimplex(origin, pointOnAToPack, pointOnBToPack);
   }
   
   public ExtendedConvexPolytope getSimplex()
   {
      return simplex.getPolytope();
   }
}
