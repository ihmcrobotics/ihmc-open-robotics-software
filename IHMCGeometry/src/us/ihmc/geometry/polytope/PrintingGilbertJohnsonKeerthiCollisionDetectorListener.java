package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

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
   public void addedVertexToSimplex(SimplexPolytope simplex, Point3D vertexOnSimplex, Point3D vertexOnA, Point3D vertexOnB)
   {
      System.out.println("\nAdded Vertex to Simplex. New simplex is: ");
      System.out.println(simplex);

   }

   @Override
   public void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3D closestPointToOrigin)
   {
      System.out.println("\nFound closest point on Simplex: " + closestPointToOrigin);
      System.out.println("Distance to origin is " + new Vector3D(closestPointToOrigin).length());
   }

   @Override
   public void foundCollision(SimplexPolytope simplex, Point3D pointOnA, Point3D pointOnB)
   {
      System.out.println("\nFound a collision!");
      System.out.println("PointOnA: " + pointOnA);
      System.out.println("PointOnB: " + pointOnB);
   }

   @Override
   public void foundSupportPoints(SimplexPolytope simplex, Point3D supportingVertexOnA, Point3D supportingVertexOnB, Vector3D supportingVertexOnSimplex)
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
   public void metStoppingConditionForNoIntersection(double vDotP, double percentCloser, Point3D closestPointOnA, Point3D closestPointOnB)
   {
      System.out.println("\nMet Stopping Condition For No Intersection");
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);

      System.out.println("---------------------------");
      System.out.println("");
   }

   @Override
   public void tooManyIterationsStopping(SimplexPolytope simplex, Point3D closestPointOnA, Point3D closestPointOnB)
   {
      System.out.println("\nToo Many Iterations. No Intersection");
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);

      System.out.println("---------------------------");
      System.out.println("");
   }

   @Override
   public void metStoppingConditionForNoIntersection(Point3D closestPointOnA, Point3D closestPointOnB)
   {
      System.out.println("\nMet Stopping Condition For No Intersection");
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);
      System.out.println("---------------------------");
      System.out.println("");

   }

}
