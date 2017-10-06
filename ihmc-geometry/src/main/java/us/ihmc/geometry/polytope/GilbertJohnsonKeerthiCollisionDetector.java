package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * GilbertJohnsonKeerthi (GJK) algorithm for doing collision detection
 *
 * For more information see book, papers, and presentations at http://realtimecollisiondetection.net/pubs/
 */
public class GilbertJohnsonKeerthiCollisionDetector
{
   private static final double LAMBDA_STOPPING_DELTA = 1e-6;

   private final Vector3D supportDirection = new Vector3D();
   private final Vector3D negativeSupportDirection = new Vector3D();
   private final Vector3D supportingVertexOnSimplex = new Vector3D();
   private final Vector3D tempPVector = new Vector3D();

   private final SimplexPolytope simplex = new SimplexPolytope();
   private GilbertJohnsonKeerthiCollisionDetectorListener listener;

   private final RecyclingArrayList<Point3D> poolOfPoints = new RecyclingArrayList<Point3D>(Point3D.class);

   public void computeSupportPointOnMinkowskiDifference(ConvexPolytope cubeOne, ConvexPolytope cubeTwo, Vector3D supportDirection, Point3D supportPoint)
   {
      // Because everything is linear and convex, the support point on the Minkowski difference is s_{a minkowskidiff b}(d) = s_a(d) - s_b(-d)

      Point3D supportingVertexOne = cubeOne.getSupportingVertex(supportDirection);

      negativeSupportDirection.set(supportDirection);
      negativeSupportDirection.scale(-1.0);

      Point3D supportingVertexTwo = cubeTwo.getSupportingVertex(negativeSupportDirection);

      supportPoint.set(supportingVertexOne);
      supportPoint.sub(supportingVertexTwo);
   }

   public void setGilbertJohnsonKeerthiCollisionDetectorListener(GilbertJohnsonKeerthiCollisionDetectorListener listener)
   {
      this.listener = listener;
   }

   private final Vector3D defaultInitialGuessOfSeparatingVector = new Vector3D(0.0, 0.0, 1.0);

   public boolean arePolytopesColliding(SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      return arePolytopesColliding(defaultInitialGuessOfSeparatingVector, polytopeA, polytopeB, pointOnAToPack, pointOnBToPack);
   }

   public boolean arePolytopesColliding(Vector3D initialGuessOfSeparatingVector, SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB,
         Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
      poolOfPoints.clear();

      if (listener != null)
      {
         listener.checkingIfPolytopesAreColliding(polytopeA, polytopeB);
      }

      simplex.clearPoints();

      // Step 1) Initialize Simplex Q to a single point in A minkowskiDifference B. Here we'll search in the direction of 
      // initialGuessOfSeparatingVector. That will ensure that the point is on the exterior of the Minkowski Difference
      // and will allow us to speed things up by remembering the previous separating vector.

      Point3D vertexOne = polytopeA.getSupportingVertex(initialGuessOfSeparatingVector);
      negativeSupportDirection.set(initialGuessOfSeparatingVector);
      negativeSupportDirection.negate();
      Point3D vertexTwo = polytopeB.getSupportingVertex(negativeSupportDirection);

      Point3D minkowskiDifferenceVertex = poolOfPoints.add();//new Point3D();
      minkowskiDifferenceVertex.sub(vertexOne, vertexTwo);

      simplex.addVertex(minkowskiDifferenceVertex, vertexOne, vertexTwo);

      if (listener != null)
      {
         listener.addedVertexToSimplex(simplex, minkowskiDifferenceVertex, vertexOne, vertexTwo);
      }

      int iterations = 0;
      int metStoppingConditionsCount = 0;
      Point3D closestPointToOrigin = poolOfPoints.add();
      closestPointToOrigin.set(0.0, 0.0, 0.0);
      Point3D origin = poolOfPoints.add();
      origin.set(0.0, 0.0, 0.0);

      while (true)
      {
         // Step 2) Compute closest point to origin in the simplex. 4) Reduce points of Q not used in determining P.
         simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);

         if (Double.isNaN(closestPointToOrigin.getX()))
         {
            throw new RuntimeException("NaN!");
         }
         if (listener != null)
         {
            listener.foundClosestPointOnSimplex(simplex, closestPointToOrigin);
         }

         // Step 3) If P is origin, done.
         double distanceSquared = closestPointToOrigin.distanceSquared(origin);
         //TODO: Magic epsilon here...
         if (distanceSquared < 1e-8)
         {
            if (listener != null)
            {
               listener.foundCollision(simplex, pointOnAToPack, pointOnBToPack);
            }

            simplex.getClosestPointsOnAAndB(pointOnAToPack, pointOnBToPack);
            return true;
         }

         // Step 5) v = support vector in negative P direction on A minkowskiDifference B.
         // In other words, it is suppport vector on A in P direction minus support vector on B in negative P direction.

         supportDirection.set(closestPointToOrigin);
         supportDirection.negate();
         Point3D supportingVertexOnA = polytopeA.getSupportingVertex(supportDirection);

         supportDirection.negate();
         Point3D supportingVertexOnB = polytopeB.getSupportingVertex(supportDirection);

         if (simplex.wereMostRecentlyDiscared(supportingVertexOnA, supportingVertexOnB))
         {
            simplex.getClosestPointsOnAAndB(pointOnAToPack, pointOnBToPack);

            if (listener != null)
            {
               listener.metStoppingConditionForNoIntersection(pointOnAToPack, pointOnBToPack);
            }

            return false;
         }
         supportingVertexOnSimplex.sub(supportingVertexOnA, supportingVertexOnB);

         if (listener != null)
         {
            listener.foundSupportPoints(simplex, supportingVertexOnA, supportingVertexOnB, supportingVertexOnSimplex);
         }

         // Step 6) If v is no more extremal in direction -P than P itself, then not intersecting. magnitude(v) is distance. Use v to determine closest points.

         tempPVector.set(closestPointToOrigin);

         double vDotP = supportingVertexOnSimplex.dot(tempPVector);
         double percentCloser = vDotP / tempPVector.dot(tempPVector);

         if (listener != null)
         {
            listener.computeVDotPAndPercentCloser(vDotP, percentCloser);
         }

         //         System.out.println("vDotP - PSquared = " + foo);

         // TODO: Do we need this epsilon here. Seems when to abort is tricky. Maybe look that the extremal points in the
         // simplex stop changing?
         if (percentCloser >= 1.0 - LAMBDA_STOPPING_DELTA)
         {
            metStoppingConditionsCount++;
         }

         // TODO: Get rid of this. Hack right now to make sure really hard cases get a full shot...
         if (metStoppingConditionsCount == 4)
         {
            simplex.getClosestPointsOnAAndB(pointOnAToPack, pointOnBToPack);

            if (listener != null)
            {
               listener.metStoppingConditionForNoIntersection(pointOnAToPack, pointOnBToPack);
            }

            return false;
         }

         iterations++;
         if (iterations > 100)
         {
            System.out.println("Seems to be looping... lambda = " + percentCloser);
            System.out.println("Seems to be looping... closestPointToOrigin = " + closestPointToOrigin);
         }

         if (iterations > 106)
         {
            simplex.getClosestPointsOnAAndB(pointOnAToPack, pointOnBToPack);

            if (listener != null)
            {
               listener.tooManyIterationsStopping(simplex, pointOnAToPack, pointOnBToPack);
            }

            return false;
         }

         // Step 7) Add v to Q and got to step 2.
         Point3D pointToAddToSimplex = poolOfPoints.add();
         pointToAddToSimplex.set(supportingVertexOnSimplex);
         boolean successfullyAddedVertex = simplex.addVertex(pointToAddToSimplex, supportingVertexOnA, supportingVertexOnB);

         if (!successfullyAddedVertex)
         {
            simplex.getClosestPointsOnAAndB(pointOnAToPack, pointOnBToPack);

            if (listener != null)
            {
               listener.metStoppingConditionForNoIntersection(pointOnAToPack, pointOnBToPack);
            }

            return false;
         }

         if (listener != null)
         {
            listener.addedVertexToSimplex(simplex, pointToAddToSimplex, supportingVertexOnA, supportingVertexOnB);
         }

      }
   }

   public SimplexPolytope getSimplex()
   {
      return simplex;
   }
}
