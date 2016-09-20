package us.ihmc.geometry.polytope;

import java.util.LinkedHashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class SimplexPolytope
{
   private Point3d pointOne, pointTwo, pointThree, pointFour;

   private LinkedHashMap<Point3d, Point3d> simplexPointToPolytopePointA = new LinkedHashMap<>();
   private LinkedHashMap<Point3d, Point3d> simplexPointToPolytopePointB = new LinkedHashMap<>();
   private LinkedHashMap<Point3d, Double> lambdas = new LinkedHashMap<>();

   public int getNumberOfPoints()
   {
      if (pointOne == null)
         return 0;
      if (pointTwo == null)
         return 1;
      if (pointThree == null)
         return 2;
      if (pointFour == null)
         return 3;
      return 4;
   }

   public void getClosestPointsOnAAndB(Point3d pointOnAToPack, Point3d pointOnBToPack)
   {
      pointOnAToPack.set(0.0, 0.0, 0.0);
      pointOnBToPack.set(0.0, 0.0, 0.0);

      if (pointOne != null)
      {
         addPointsContribution(pointOne, pointOnAToPack, pointOnBToPack);
      }
      if (pointTwo != null)
      {
         addPointsContribution(pointTwo, pointOnAToPack, pointOnBToPack);
      }
      if (pointThree != null)
      {
         addPointsContribution(pointThree, pointOnAToPack, pointOnBToPack);
      }
      if (pointFour != null)
      {
         addPointsContribution(pointFour, pointOnAToPack, pointOnBToPack);
      }
   }

   private void addPointsContribution(Point3d pointToContribute, Point3d pointOnAToPack, Point3d pointOnBToPack)
   {
      double lambda = lambdas.get(pointToContribute);

      tempVector1.set(simplexPointToPolytopePointA.get(pointToContribute));
      tempVector1.scale(lambda);
      pointOnAToPack.add(tempVector1);

      tempVector1.set(simplexPointToPolytopePointB.get(pointToContribute));
      tempVector1.scale(lambda);
      pointOnBToPack.add(tempVector1);
   }

   public void clearPoints()
   {
      pointOne = pointTwo = pointThree = pointFour = null;
   }

   public void setPoints(Point3d... points)
   {
      pointOne = pointTwo = pointThree = pointFour = null;

      if (points.length > 0)
         pointOne = points[0];
      if (points.length > 1)
         pointTwo = points[1];
      if (points.length > 2)
         pointThree = points[2];
      if (points.length > 3)
         pointFour = points[3];
   }

   public Point3d getPoint(int i)
   {
      if (i == 0)
         return pointOne;
      if (i == 1)
         return pointTwo;
      if (i == 2)
         return pointThree;
      if (i == 3)
         return pointFour;

      return null;
   }

   public double getLambda(Point3d point)
   {
      return lambdas.get(point);
   }

   public boolean addVertex(Point3d simplexPointToAdd, Point3d correspondingPointOnA, Point3d correspondingPointOnB)
   {
      boolean alreadyHaveThatOne = doWeAlreadyHaveThatVertex(correspondingPointOnA, correspondingPointOnB);

      if (alreadyHaveThatOne)
         return false;

      if (pointOne == null)
      {
         pointOne = simplexPointToAdd;
      }
      else if (pointTwo == null)
      {
         pointTwo = simplexPointToAdd;
      }
      else if (pointThree == null)
      {
         pointThree = simplexPointToAdd;
      }
      else if (pointFour == null)
      {
         pointFour = simplexPointToAdd;
      }
      else
         throw new RuntimeException("Only support SimplexPolytopes with at most 4 vertices");

      simplexPointToPolytopePointA.put(simplexPointToAdd, correspondingPointOnA);
      simplexPointToPolytopePointB.put(simplexPointToAdd, correspondingPointOnB);

      return true;
   }

   private boolean doWeAlreadyHaveThatVertex(Point3d correspondingPointOnA, Point3d correspondingPointOnB)
   {
      if (pointOne != null)
      {
         if ((correspondingPointOnA == simplexPointToPolytopePointA.get(pointOne)) && (correspondingPointOnB == simplexPointToPolytopePointB.get(pointOne)))
            return true;
      }
      if (pointTwo != null)
      {
         if ((correspondingPointOnA == simplexPointToPolytopePointA.get(pointTwo)) && (correspondingPointOnB == simplexPointToPolytopePointB.get(pointTwo)))
            return true;
      }
      if (pointThree != null)
      {
         if ((correspondingPointOnA == simplexPointToPolytopePointA.get(pointThree)) && (correspondingPointOnB == simplexPointToPolytopePointB.get(pointThree)))
            return true;
      }
      if (pointFour != null)
      {
         if ((correspondingPointOnA == simplexPointToPolytopePointA.get(pointFour)) && (correspondingPointOnB == simplexPointToPolytopePointB.get(pointFour)))
            return true;
      }

      return false;
   }

   public boolean containsPoint(Point3d pointToCheck)
   {
      if (pointToCheck == pointOne)
         return true;
      if (pointToCheck == pointTwo)
         return true;
      if (pointToCheck == pointThree)
         return true;
      if (pointToCheck == pointFour)
         return true;

      return false;
   }

   public void removePoint(Point3d pointToRemove)
   {
      if (pointToRemove == pointOne)
      {
         pointOne = pointTwo;
         pointTwo = pointThree;
         pointThree = pointFour;
         pointFour = null;
      }

      else if (pointToRemove == pointTwo)
      {
         pointTwo = pointThree;
         pointThree = pointFour;
         pointFour = null;
      }

      else if (pointToRemove == pointThree)
      {
         pointThree = pointFour;
         pointFour = null;
      }

      else if (pointToRemove == pointFour)
      {
         pointFour = null;
      }

   }

   public void getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(Point3d closestPointToOrigin)
   {
      // Use the distance subalgorithm of GJK and go through all the possibilities:

      forgetDiscardedVertices();

      if (pointOne == null)
      {
         throw new RuntimeException();
      }

      else if (pointTwo == null)
      {
         projectOriginOntoPoint(pointOne, closestPointToOrigin);
         retainPoints(pointOne);
      }

      else if (pointThree == null)
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo))
         {
            projectOriginOntoPoint(pointOne, closestPointToOrigin);
            retainPoints(pointOne);
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne))
         {
            projectOriginOntoPoint(pointTwo, closestPointToOrigin);
            retainPoints(pointTwo);
         }
         else
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            retainPoints(pointOne, pointTwo);
         }
      }

      else if (pointFour == null)
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree))
         {
            projectOriginOntoPoint(pointOne, closestPointToOrigin);
            retainPoints(pointOne);
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree))
         {
            projectOriginOntoPoint(pointTwo, closestPointToOrigin);
            retainPoints(pointTwo);
         }
         else if (isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo))
         {
            projectOriginOntoPoint(pointThree, closestPointToOrigin);
            retainPoints(pointThree);
         }

         else if (isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree))
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            retainPoints(pointOne, pointTwo);
         }
         else if (isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo))
         {
            projectOriginOntoEdge(pointOne, pointThree, closestPointToOrigin);
            retainPoints(pointOne, pointThree);
         }
         else if (isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne))
         {
            projectOriginOntoEdge(pointTwo, pointThree, closestPointToOrigin);
            retainPoints(pointTwo, pointThree);
         }
         else
         {
            projectOriginOntoFace(pointOne, pointTwo, pointThree, closestPointToOrigin);
            retainPoints(pointOne, pointTwo, pointThree);
         }
      }

      else
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree, pointFour))
         {
            projectOriginOntoPoint(pointOne, closestPointToOrigin);
            retainPoints(pointOne);
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree, pointFour))
         {
            projectOriginOntoPoint(pointTwo, closestPointToOrigin);
            retainPoints(pointTwo);
         }
         else if (isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo, pointFour))
         {
            projectOriginOntoPoint(pointThree, closestPointToOrigin);
            retainPoints(pointThree);
         }
         else if (isInVoronoiRegionOfVertex(pointFour, pointOne, pointTwo, pointThree))
         {
            projectOriginOntoPoint(pointFour, closestPointToOrigin);
            retainPoints(pointFour);
         }

         else if (isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree, pointFour))
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            retainPoints(pointOne, pointTwo);
         }
         else if (isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo, pointFour))
         {
            projectOriginOntoEdge(pointOne, pointThree, closestPointToOrigin);
            retainPoints(pointOne, pointThree);
         }
         else if (isInVoronoiRegionOfEdge(pointOne, pointFour, pointTwo, pointThree))
         {
            projectOriginOntoEdge(pointOne, pointFour, closestPointToOrigin);
            retainPoints(pointOne, pointFour);
         }
         else if (isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne, pointFour))
         {
            projectOriginOntoEdge(pointTwo, pointThree, closestPointToOrigin);
            retainPoints(pointTwo, pointThree);
         }
         else if (isInVoronoiRegionOfEdge(pointTwo, pointFour, pointOne, pointThree))
         {
            projectOriginOntoEdge(pointTwo, pointFour, closestPointToOrigin);
            retainPoints(pointTwo, pointFour);
         }
         else if (isInVoronoiRegionOfEdge(pointThree, pointFour, pointOne, pointTwo))
         {
            projectOriginOntoEdge(pointThree, pointFour, closestPointToOrigin);
            retainPoints(pointThree, pointFour);
         }

         else
         {

            // TODO: Should not have to check the lambdas with lambdasAreOK() call!
            // Not sure why some of the harder, nearly coplanar are such that tow of these
            // Face Voronoi region checks are true. Only one should be true if
            // the edges and vertices are not Voronoi regions.
            // Or maybe I have something wrong with the math?

            if (isInVoronoiRegionOfFace(pointOne, pointTwo, pointThree, pointFour))
            {
               projectOriginOntoFace(pointOne, pointTwo, pointThree, closestPointToOrigin);
               if (lambdasAreOK())
               {
                  rememberDiscardedVertices(pointFour);
                  retainPoints(pointOne, pointTwo, pointThree);
                  return;
               }
            }

            if (isInVoronoiRegionOfFace(pointOne, pointTwo, pointFour, pointThree))
            {
               projectOriginOntoFace(pointOne, pointTwo, pointFour, closestPointToOrigin);
               if (lambdasAreOK())
               {
                  rememberDiscardedVertices(pointThree);
                  retainPoints(pointOne, pointTwo, pointFour);
                  return;
               }
            }

            if (isInVoronoiRegionOfFace(pointOne, pointThree, pointFour, pointTwo))
            {
               projectOriginOntoFace(pointOne, pointThree, pointFour, closestPointToOrigin);
               if (lambdasAreOK())
               {
                  rememberDiscardedVertices(pointTwo);
                  retainPoints(pointOne, pointThree, pointFour);
                  return;
               }
            }

            if (isInVoronoiRegionOfFace(pointTwo, pointThree, pointFour, pointOne))
            {
               projectOriginOntoFace(pointTwo, pointThree, pointFour, closestPointToOrigin);
               if (lambdasAreOK())
               {
                  rememberDiscardedVertices(pointOne);
                  retainPoints(pointTwo, pointThree, pointFour);
                  return;
               }
            }

            {
               projectInsideTetragon(closestPointToOrigin);

               //TODO: The retain points when you don't throw any away should be unnecessary...
               forgetDiscardedVertices();
               retainPoints(pointOne, pointTwo, pointThree, pointFour);
            }
         }
      }
   }

   private boolean lambdasAreOK()
   {
      if (lambdas.containsKey(pointOne))
      {
         double lambda = lambdas.get(pointOne);
         if ((lambda < 0.0) || (lambda > 1.0))
            return false;
      }

      if (lambdas.containsKey(pointTwo))
      {
         double lambda = lambdas.get(pointTwo);
         if ((lambda < 0.0) || (lambda > 1.0))
            return false;
      }

      if (lambdas.containsKey(pointThree))
      {
         double lambda = lambdas.get(pointThree);
         if ((lambda < 0.0) || (lambda > 1.0))
            return false;
      }

      if (lambdas.containsKey(pointFour))
      {
         double lambda = lambdas.get(pointFour);
         if ((lambda < 0.0) || (lambda > 1.0))
            return false;
      }

      return true;
   }

   private void projectInsideTetragon(Point3d closestPointToOrigin)
   {
      // Compute barycentric coordinates for point inside the tetragon.

      DenseMatrix64F tetragonMatrix = new DenseMatrix64F(4, 4);

      tetragonMatrix.set(0, 0, pointOne.getX());
      tetragonMatrix.set(1, 0, pointOne.getY());
      tetragonMatrix.set(2, 0, pointOne.getZ());
      tetragonMatrix.set(3, 0, 1.0);

      tetragonMatrix.set(0, 1, pointTwo.getX());
      tetragonMatrix.set(1, 1, pointTwo.getY());
      tetragonMatrix.set(2, 1, pointTwo.getZ());
      tetragonMatrix.set(3, 1, 1.0);

      tetragonMatrix.set(0, 2, pointThree.getX());
      tetragonMatrix.set(1, 2, pointThree.getY());
      tetragonMatrix.set(2, 2, pointThree.getZ());
      tetragonMatrix.set(3, 2, 1.0);

      tetragonMatrix.set(0, 3, pointFour.getX());
      tetragonMatrix.set(1, 3, pointFour.getY());
      tetragonMatrix.set(2, 3, pointFour.getZ());
      tetragonMatrix.set(3, 3, 1.0);

      DenseMatrix64F tetragonVector = new DenseMatrix64F(4, 1);
      tetragonVector.set(0, 0, 0.0);
      tetragonVector.set(1, 0, 0.0);
      tetragonVector.set(2, 0, 0.0);
      tetragonVector.set(3, 0, 1.0);

      DenseMatrix64F tetragonLambdas = new DenseMatrix64F(4, 1);
      CommonOps.solve(tetragonMatrix, tetragonVector, tetragonLambdas);

      lambdas.clear();
      setLambda(pointOne, tetragonLambdas.get(0, 0));
      setLambda(pointTwo, tetragonLambdas.get(1, 0));
      setLambda(pointThree, tetragonLambdas.get(2, 0));
      setLambda(pointFour, tetragonLambdas.get(3, 0));

      closestPointToOrigin.set(0.0, 0.0, 0.0);
   }

   private void retainPoints(Point3d pointToKeep)
   {
      Point3d point3dOnA = simplexPointToPolytopePointA.get(pointToKeep);
      Point3d point3dOnB = simplexPointToPolytopePointB.get(pointToKeep);

      pointOne = null;
      pointTwo = null;
      pointThree = null;
      pointFour = null;

      pointOne = pointToKeep;

      simplexPointToPolytopePointA.clear();
      simplexPointToPolytopePointB.clear();

      simplexPointToPolytopePointA.put(pointToKeep, point3dOnA);
      simplexPointToPolytopePointB.put(pointToKeep, point3dOnB);
   }

   private void retainPoints(Point3d pointToKeep1, Point3d pointToKeep2)
   {
      Point3d point3dOnA1 = simplexPointToPolytopePointA.get(pointToKeep1);
      Point3d point3dOnA2 = simplexPointToPolytopePointA.get(pointToKeep2);

      Point3d point3dOnB1 = simplexPointToPolytopePointB.get(pointToKeep1);
      Point3d point3dOnB2 = simplexPointToPolytopePointB.get(pointToKeep2);

      pointOne = null;
      pointTwo = null;
      pointThree = null;
      pointFour = null;

      pointOne = pointToKeep1;
      pointTwo = pointToKeep2;

      simplexPointToPolytopePointA.clear();
      simplexPointToPolytopePointB.clear();

      simplexPointToPolytopePointA.put(pointToKeep1, point3dOnA1);
      simplexPointToPolytopePointA.put(pointToKeep2, point3dOnA2);
      simplexPointToPolytopePointB.put(pointToKeep1, point3dOnB1);
      simplexPointToPolytopePointB.put(pointToKeep2, point3dOnB2);
   }

   private void retainPoints(Point3d pointToKeep1, Point3d pointToKeep2, Point3d pointToKeep3)
   {
      Point3d point3dOnA1 = simplexPointToPolytopePointA.get(pointToKeep1);
      Point3d point3dOnA2 = simplexPointToPolytopePointA.get(pointToKeep2);
      Point3d point3dOnA3 = simplexPointToPolytopePointA.get(pointToKeep3);

      Point3d point3dOnB1 = simplexPointToPolytopePointB.get(pointToKeep1);
      Point3d point3dOnB2 = simplexPointToPolytopePointB.get(pointToKeep2);
      Point3d point3dOnB3 = simplexPointToPolytopePointB.get(pointToKeep3);

      pointOne = null;
      pointTwo = null;
      pointThree = null;
      pointFour = null;

      pointOne = pointToKeep1;
      pointTwo = pointToKeep2;
      pointThree = pointToKeep3;

      simplexPointToPolytopePointA.clear();
      simplexPointToPolytopePointB.clear();

      simplexPointToPolytopePointA.put(pointToKeep1, point3dOnA1);
      simplexPointToPolytopePointA.put(pointToKeep2, point3dOnA2);
      simplexPointToPolytopePointA.put(pointToKeep3, point3dOnA3);
      simplexPointToPolytopePointB.put(pointToKeep1, point3dOnB1);
      simplexPointToPolytopePointB.put(pointToKeep2, point3dOnB2);
      simplexPointToPolytopePointB.put(pointToKeep3, point3dOnB3);
   }

   private void retainPoints(Point3d pointToKeep1, Point3d pointToKeep2, Point3d pointToKeep3, Point3d pointToKeep4)
   {
      Point3d point3dOnA1 = simplexPointToPolytopePointA.get(pointToKeep1);
      Point3d point3dOnA2 = simplexPointToPolytopePointA.get(pointToKeep2);
      Point3d point3dOnA3 = simplexPointToPolytopePointA.get(pointToKeep3);
      Point3d point3dOnA4 = simplexPointToPolytopePointA.get(pointToKeep4);

      Point3d point3dOnB1 = simplexPointToPolytopePointB.get(pointToKeep1);
      Point3d point3dOnB2 = simplexPointToPolytopePointB.get(pointToKeep2);
      Point3d point3dOnB3 = simplexPointToPolytopePointB.get(pointToKeep3);
      Point3d point3dOnB4 = simplexPointToPolytopePointB.get(pointToKeep4);

      pointOne = null;
      pointTwo = null;
      pointThree = null;
      pointFour = null;

      pointOne = pointToKeep1;
      pointTwo = pointToKeep2;
      pointThree = pointToKeep3;
      pointFour = pointToKeep4;

      simplexPointToPolytopePointA.clear();
      simplexPointToPolytopePointB.clear();

      simplexPointToPolytopePointA.put(pointToKeep1, point3dOnA1);
      simplexPointToPolytopePointA.put(pointToKeep2, point3dOnA2);
      simplexPointToPolytopePointA.put(pointToKeep3, point3dOnA3);
      simplexPointToPolytopePointA.put(pointToKeep4, point3dOnA4);

      simplexPointToPolytopePointB.put(pointToKeep1, point3dOnB1);
      simplexPointToPolytopePointB.put(pointToKeep2, point3dOnB2);
      simplexPointToPolytopePointB.put(pointToKeep3, point3dOnB3);
      simplexPointToPolytopePointB.put(pointToKeep4, point3dOnB4);
   }

   private Point3d discardedOnA, discardedOnB;

   private void rememberDiscardedVertices(Point3d pointToDiscard)
   {
      discardedOnA = simplexPointToPolytopePointA.get(pointToDiscard);
      discardedOnB = simplexPointToPolytopePointB.get(pointToDiscard);
   }

   private void forgetDiscardedVertices()
   {
      discardedOnA = discardedOnB = null;
   }

   public boolean wereMostRecentlyDiscared(Point3d checkOnA, Point3d checkOnB)
   {
      return ((discardedOnA == checkOnA) && (discardedOnB == checkOnB));
   }

   public boolean isInVoronoiRegionOfVertex(Point3d pointToCheck, Point3d otherPoint)
   {
      tempVector1.set(pointToCheck);
      tempVector1.scale(-1.0);

      tempVector2.sub(otherPoint, pointToCheck);
      return (tempVector1.dot(tempVector2) <= 0.0);
   }

   public boolean isInVoronoiRegionOfVertex(Point3d pointToCheck, Point3d otherPoint1, Point3d otherPoint2)
   {
      tempVector1.set(pointToCheck);
      tempVector1.scale(-1.0);

      tempVector2.sub(otherPoint1, pointToCheck);
      if (!(tempVector1.dot(tempVector2) <= 0.0))
         return false;

      tempVector2.sub(otherPoint2, pointToCheck);
      if (!(tempVector1.dot(tempVector2) <= 0.0))
         return false;

      return true;
   }

   public boolean isInVoronoiRegionOfVertex(Point3d pointToCheck, Point3d otherPoint1, Point3d otherPoint2, Point3d otherPoint3)
   {
      tempVector1.set(pointToCheck);
      tempVector1.scale(-1.0);

      tempVector2.sub(otherPoint1, pointToCheck);
      double dotProduct = tempVector1.dot(tempVector2);
      if (!(dotProduct <= 0.0))
         return false;

      tempVector2.sub(otherPoint2, pointToCheck);
      dotProduct = tempVector1.dot(tempVector2);
      if (!(dotProduct <= 0.0))
         return false;

      tempVector2.sub(otherPoint3, pointToCheck);
      dotProduct = tempVector1.dot(tempVector2);
      if (!(dotProduct <= 0.0))
         return false;

      return true;
   }

   private final Vector3d tempVector1 = new Vector3d();
   private final Vector3d tempVector2 = new Vector3d();
   private final Vector3d tempVector3 = new Vector3d();
   private final Vector3d tempVector4 = new Vector3d();
   private final Vector3d tempNormalVector1 = new Vector3d();
   private final Vector3d tempNormalVector2 = new Vector3d();

   public boolean isInVoronoiRegionOfEdge(Point3d edgePointOne, Point3d edgePointTwo, Point3d otherPoint)
   {
      // TODO: Redundancies from other checks that be reused. Or is this just bad, due to epsilon problems.
      // If we are here, then we know we failed one or more of the Vertex Voronoi region checks (but we do not know which ones...).
      //So maybe these should be removed?
      tempVector1.set(edgePointOne);
      tempVector1.scale(-1.0);

      tempVector2.sub(edgePointTwo, edgePointOne);
      if (!(tempVector1.dot(tempVector2) >= 0.0))
         return false;

      tempVector1.set(edgePointTwo);
      tempVector1.scale(-1.0);

      tempVector2.sub(edgePointOne, edgePointTwo);
      if (!(tempVector1.dot(tempVector2) >= 0.0))
         return false;

      ////////

      tempVector1.sub(edgePointTwo, edgePointOne);
      tempVector2.sub(otherPoint, edgePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector2.cross(tempVector1, tempNormalVector1);
      tempVector3.set(edgePointOne);
      tempVector3.scale(-1.0);
      return (tempVector3.dot(tempVector2) >= 0.0);
   }

   public boolean isInVoronoiRegionOfEdge(Point3d edgePointOne, Point3d edgePointTwo, Point3d otherPointOne, Point3d otherPointTwo)
   {
      // TODO: Redundancies from other checks that be reused...
      tempVector1.set(edgePointOne);
      tempVector1.scale(-1.0);

      tempVector2.sub(edgePointTwo, edgePointOne);
      double dotProduct = tempVector1.dot(tempVector2);
      if (!(dotProduct >= 0.0))
         return false;

      tempVector1.set(edgePointTwo);
      tempVector1.scale(-1.0);

      tempVector2.sub(edgePointOne, edgePointTwo);
      dotProduct = tempVector1.dot(tempVector2);
      if (!(dotProduct >= 0.0))
         return false;

      ////////

      tempVector1.sub(edgePointTwo, edgePointOne);
      tempVector2.sub(otherPointOne, edgePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector2.cross(tempVector1, tempNormalVector1);
      tempVector3.set(edgePointOne);
      tempVector3.scale(-1.0);
      dotProduct = tempVector3.dot(tempVector2);
      if (!(dotProduct >= 0.0))
         return false;

      tempVector1.sub(edgePointTwo, edgePointOne);
      tempVector2.sub(otherPointTwo, edgePointOne);

      tempNormalVector2.cross(tempVector2, tempVector1);

      tempVector2.cross(tempNormalVector2, tempVector1);
      tempVector3.set(edgePointOne);
      tempVector3.scale(-1.0);
      dotProduct = tempVector3.dot(tempVector2);
      if (!(dotProduct >= 0.0))
         return false;

      return true;
   }

   public boolean isInVoronoiRegionOfFace(Point3d facePointOne, Point3d facePointTwo, Point3d facePointThree, Point3d otherPoint)
   {
      tempVector1.sub(facePointTwo, facePointOne);
      tempVector2.sub(facePointThree, facePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector1.set(facePointOne);
      tempVector1.scale(-1.0);
      double dot1 = tempVector1.dot(tempNormalVector1);

      tempVector3.sub(otherPoint, facePointOne);
      double dot2 = tempVector3.dot(tempNormalVector1);

      // TODO: Magic delta here. Figure out robustness to deltas.
      if (Math.abs(dot2) < 1e-10)//1e-6)
         return true; // Other point is in the same plane. Just project to the plane then.
      return (dot1 * dot2 < 0.0);

      //      return (dot1 * dot2 < 0.0 + 1e-8);
   }

   private void setLambda(Point3d vertex, double lambda)
   {
      if (lambda < 0.0)
      {
         //         System.err.println("Trouble with simplex:");
         //         System.err.println(this);
         //         throw new RuntimeException("lambda < 0.0! lambda = " + lambda);
      }
      if (lambda > 1.0)
      {
         //         System.err.println("Trouble with simplex:");
         //         System.err.println(this);
         //         throw new RuntimeException("lambda > 1.0! lambda = " + lambda);
      }

      lambdas.put(vertex, lambda);
   }

   private void projectOriginOntoPoint(Point3d vertex, Point3d projectionToPack)
   {
      projectionToPack.set(vertex);

      lambdas.clear();
      setLambda(vertex, 1.0);
   }

   private void projectOriginOntoEdge(Point3d vertexOne, Point3d vertexTwo, Point3d projectionToPack)
   {
      tempVector1.set(vertexOne);
      tempVector1.scale(-1.0);
      tempVector2.sub(vertexTwo, vertexOne);

      double percentFromVertexOneToVertexTwo = tempVector1.dot(tempVector2) / (tempVector2.dot(tempVector2));
      tempVector2.scale(percentFromVertexOneToVertexTwo);

      projectionToPack.set(vertexOne);
      projectionToPack.add(tempVector2);

      double oneMinusPercentFromVertexOneToVertexTwo = 1.0 - percentFromVertexOneToVertexTwo;

      lambdas.clear();
      setLambda(vertexOne, oneMinusPercentFromVertexOneToVertexTwo);
      setLambda(vertexTwo, percentFromVertexOneToVertexTwo);

   }

   //   private void projectOriginOntoFace(Point3d vertexOne, Point3d vertexTwo, Point3d vertexThree, Point3d closestPointToOrigin)
   //   {
   //      tempVector1.sub(vertexTwo, vertexOne);
   //      tempVector2.sub(vertexThree, vertexOne);
   //
   //      tempNormalVector1.cross(tempVector1, tempVector2);
   //      tempNormalVector1.normalize();
   //
   //      tempVector3.set(vertexOne);
   //      tempVector3.scale(-1.0);
   //
   //      double dot = tempVector3.dot(tempNormalVector1);
   //      closestPointToOrigin.set(tempNormalVector1);
   //      closestPointToOrigin.scale(-dot);
   //
   //
   //   }

   private void projectOriginOntoFace(Point3d vertexOne, Point3d vertexTwo, Point3d vertexThree, Point3d closestPointToOrigin)
   {
      // Using barycentric coordinates as described in https://www.cs.ubc.ca/~heidrich/Papers/JGT.05.pdf
      tempVector1.sub(vertexTwo, vertexOne);
      tempVector2.sub(vertexThree, vertexOne);

      tempNormalVector1.cross(tempVector1, tempVector2);
      double oneOver4ASquared = 1.0 / (tempNormalVector1.dot(tempNormalVector1));

      tempVector3.set(vertexOne);
      tempVector3.scale(-1.0); //w

      tempVector4.cross(tempVector1, tempVector3);
      double lambdaThree = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      tempVector4.cross(tempVector3, tempVector2);
      double lambdaTwo = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      double lambdaOne = 1.0 - lambdaTwo - lambdaThree;

      lambdas.clear();
      setLambda(vertexOne, lambdaOne);
      setLambda(vertexTwo, lambdaTwo);
      setLambda(vertexThree, lambdaThree);

      closestPointToOrigin.set(0.0, 0.0, 0.0);

      tempVector1.set(vertexOne);
      tempVector1.scale(lambdaOne);
      closestPointToOrigin.add(tempVector1);

      tempVector1.set(vertexTwo);
      tempVector1.scale(lambdaTwo);
      closestPointToOrigin.add(tempVector1);

      tempVector1.set(vertexThree);
      tempVector1.scale(lambdaThree);
      closestPointToOrigin.add(tempVector1);
   }

   public Point3d getCorrespondingPointOnPolytopeA(Point3d simplexPoint)
   {
      return simplexPointToPolytopePointA.get(simplexPoint);
   }

   public Point3d getCorrespondingPointOnPolytopeB(Point3d simplexPoint)
   {
      return simplexPointToPolytopePointB.get(simplexPoint);
   }

   public String toString()
   {
      String string = "";

      if (pointOne != null)
      {
         string = string + pointOne + ": " + simplexPointToPolytopePointA.get(pointOne) + ", " + simplexPointToPolytopePointB.get(pointOne);
      }
      if (pointTwo != null)
      {
         string = string + "\n" + pointTwo + ": " + simplexPointToPolytopePointA.get(pointTwo) + ", " + simplexPointToPolytopePointB.get(pointTwo);
      }
      if (pointThree != null)
      {
         string = string + "\n" + pointThree + ": " + simplexPointToPolytopePointA.get(pointThree) + ", " + simplexPointToPolytopePointB.get(pointThree);
      }
      if (pointFour != null)
      {
         string = string + "\n" + pointFour + ": " + simplexPointToPolytopePointA.get(pointFour) + ", " + simplexPointToPolytopePointB.get(pointFour);
      }

      return string;
   }

   public double computeTripleProductIfTetragon()
   {
      if (pointFour == null)
         return Double.NaN;

      Vector3d vectorAB = new Vector3d();
      Vector3d vectorAC = new Vector3d();
      Vector3d vectorAD = new Vector3d();
      Vector3d normalVector = new Vector3d();

      vectorAB.sub(pointTwo, pointOne);
      vectorAC.sub(pointThree, pointOne);
      vectorAD.sub(pointFour, pointOne);
      normalVector.cross(vectorAB, vectorAC);
      double tripleProduct = vectorAD.dot(normalVector);

      return tripleProduct;
   }

   public boolean hasFourCoplanarPoints()
   {
      if (pointFour == null)
         return false;

      double tripleProduct = computeTripleProductIfTetragon();
      if (tripleProduct == 0.0)
         return true;

      return false;
   }

}
