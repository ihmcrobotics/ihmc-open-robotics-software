package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class GilbertJohnsonKeerthiCollisionDetectorAssertListener implements GilbertJohnsonKeerthiCollisionDetectorListener
{

   @Override
   public void checkingIfPolytopesAreColliding(SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void addedVertexToSimplex(SimplexPolytope simplex, Point3D vertexOnSimplex, Point3D vertexOnA, Point3D vertexOnB)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3D closestPointToOrigin)
   {
      int numberOfPoints = simplex.getNumberOfPoints();

      double lambdaTotal = 0.0;

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = simplex.getPoint(i);
         double lambda = simplex.getLambda(point);

         if ((lambda < 0.0 - 1e-7) || (lambda > 1.0 + 1e-7))
         {
            System.err.println("\n---------------------\nlambda = " + lambda + ". Troublesome simplex for closest point:");
            System.err.println(simplex);
         }
         
         boolean lambdaGreaterThanZero = lambda >= 0.0 - 1e-7;
         boolean lambdaLessThanOne = lambda <= 1.0 + 1e-7;

         if (!lambdaGreaterThanZero)
         {
            System.err.println("lambda = " + lambda + "\nTroublesome simplex: \n" + simplex);
         }

         if (!lambdaLessThanOne)
         {
            System.err.println("lambda = " + lambda + "\nTroublesome simplex: \n" + simplex);
         }

         assertTrue(lambdaGreaterThanZero);
         assertTrue(lambdaLessThanOne);
         lambdaTotal = lambdaTotal + lambda;
      }

      assertEquals(1.0, lambdaTotal, 1e-7);

      for (int i = numberOfPoints; i < 4; i++)
      {
         Point3D point = simplex.getPoint(i);
         assertNull(point);
      }
   }

   @Override
   public void foundCollision(SimplexPolytope simplex, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void foundSupportPoints(SimplexPolytope simplex, Point3D supportingPointOnA, Point3D supportingPointOnB, Vector3D supportPointOnSimplex)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void computeVDotPAndPercentCloser(double vDotP, double percentCloser)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void metStoppingConditionForNoIntersection(double vDotP, double percentCloser, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void tooManyIterationsStopping(SimplexPolytope simplex, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void metStoppingConditionForNoIntersection(Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

}
