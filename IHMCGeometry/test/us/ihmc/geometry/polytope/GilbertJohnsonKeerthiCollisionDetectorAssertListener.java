package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class GilbertJohnsonKeerthiCollisionDetectorAssertListener implements GilbertJohnsonKeerthiCollisionDetectorListener
{

   @Override
   public void checkingIfPolytopesAreColliding(SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void addedVertexToSimplex(SimplexPolytope simplex, Point3d vertexOnSimplex, Point3d vertexOnA, Point3d vertexOnB)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3d closestPointToOrigin)
   {
      int numberOfPoints = simplex.getNumberOfPoints();

      double lambdaTotal = 0.0;

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3d point = simplex.getPoint(i);
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
         Point3d point = simplex.getPoint(i);
         assertNull(point);
      }
   }

   @Override
   public void foundCollision(SimplexPolytope simplex, Point3d pointOnAToPack, Point3d pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void foundSupportPoints(SimplexPolytope simplex, Point3d supportingPointOnA, Point3d supportingPointOnB, Vector3d supportPointOnSimplex)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void computeVDotPAndPercentCloser(double vDotP, double percentCloser)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void metStoppingConditionForNoIntersection(double vDotP, double percentCloser, Point3d pointOnAToPack, Point3d pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void tooManyIterationsStopping(SimplexPolytope simplex, Point3d pointOnAToPack, Point3d pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void metStoppingConditionForNoIntersection(Point3d pointOnAToPack, Point3d pointOnBToPack)
   {
      // TODO Auto-generated method stub

   }

}
