package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class PrintingGilbertJohnsonKeerthiCollisionDetectorListener implements GilbertJohnsonKeerthiCollisionDetectorListener
{

   @Override
   public void checkingIfPolytopesAreColliding(SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB)
   {
      System.out.println("\nChecking if Polytopes are colliding");
      System.out.println("PolytopeA = \n" + polytopeA);
      System.out.println("PolytopeB = \n" + polytopeB);
   }

   @Override
   public void addedVertexToSimplex(SimplexPolytope simplex, Point3d vertexOnSimplex, Point3d vertexOnA, Point3d vertexOnB)
   {
      System.out.println("\nAdded Vertex to Simplex. New simplex is: ");
      System.out.println(simplex);

   }

   @Override
   public void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3d closestPointToOrigin)
   {
      System.out.println("\nFound closest point on Simplex: " + closestPointToOrigin);
      System.out.println("Distance to origin is " + new Vector3d(closestPointToOrigin).length());
   }

   @Override
   public void foundCollision(SimplexPolytope simplex, Point3d pointOnA, Point3d pointOnB)
   {
      System.out.println("\nFound a collision!");
      System.out.println("PointOnA: " + pointOnA);
      System.out.println("PointOnB: " + pointOnB);
   }

   @Override
   public void foundSupportPoints(SimplexPolytope simplex, Point3d supportingVertexOnA, Point3d supportingVertexOnB, Vector3d supportingVertexOnSimplex)
   {
      System.out.println("\nFound supporting vertices");
      System.out.println("supportingVertexOnA: " + supportingVertexOnA);
      System.out.println("supportingVertexOnB: " + supportingVertexOnB);
      System.out.println("supportingVertexOnSimplex: " + supportingVertexOnSimplex);
   }

   @Override
   public void computeVDotPAndPercentCloser(double vDotP, double percentCloser)
   {
      System.out.println("\nvDotP = " + vDotP);
      System.out.println("percentCloser = " + percentCloser);
   }

   @Override
   public void metStoppingConditionForNoIntersection(double vDotP, double percentCloser, Point3d closestPointOnA, Point3d closestPointOnB)
   {
      System.out.println("\nMet Stopping Condition For No Intersection");
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);

      System.out.println("---------------------------");
      System.out.println("");
   }

   @Override
   public void tooManyIterationsStopping(SimplexPolytope simplex, Point3d closestPointOnA, Point3d closestPointOnB)
   {
      System.out.println("\nToo Many Iterations. No Intersection");
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);

      System.out.println("---------------------------");
      System.out.println("");
   }

   @Override
   public void metStoppingConditionForNoIntersection(Point3d closestPointOnA, Point3d closestPointOnB)
   {
      System.out.println("\nMet Stopping Condition For No Intersection");
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);
      System.out.println("---------------------------");
      System.out.println("");

   }

}
