package us.ihmc.geometry.polytope;

import java.util.LinkedHashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

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
      pointOne = pointTwo = pointThree = pointFour;
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

   public void addPoint(Point3d simplexPointToAdd, Point3d correspondingPointOnA, Point3d correspondingPointOnB)
   {
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

      if (pointOne == null)
      {
         throw new RuntimeException();
      }

      else if (pointTwo == null)
      {
         closestPointToOrigin.set(pointOne);
         retainPoints(pointOne);
      }

      else if (pointThree == null)
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo))
         {
            closestPointToOrigin.set(pointOne);
            retainPoints(pointOne);
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne))
         {
            closestPointToOrigin.set(pointTwo);
            retainPoints(pointTwo);
         }
         else
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
         }
      }

      else if (pointFour == null)
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree))
         {
            closestPointToOrigin.set(pointOne);
            retainPoints(pointOne);
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree))
         {
            closestPointToOrigin.set(pointTwo);
            retainPoints(pointTwo);
         }
         else if (isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo))
         {
            closestPointToOrigin.set(pointThree);
            retainPoints(pointThree);
         }

         else if (isInVornoiRegionOfEdge(pointOne, pointTwo, pointThree))
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            retainPoints(pointOne, pointTwo);
         }
         else if (isInVornoiRegionOfEdge(pointOne, pointThree, pointTwo))
         {
            projectOriginOntoEdge(pointOne, pointThree, closestPointToOrigin);
            retainPoints(pointOne, pointThree);
         }
         else if (isInVornoiRegionOfEdge(pointTwo, pointThree, pointOne))
         {
            projectOriginOntoEdge(pointTwo, pointThree, closestPointToOrigin);
            retainPoints(pointTwo, pointThree);
         }
         else
         {
            //TODO: Compute lambdas for the case of intersection
            closestPointToOrigin.set(0.0, 0.0, 0.0);

            lambdas.clear();
            lambdas.put(pointOne, 0.0);
            lambdas.put(pointTwo, 0.0);
            lambdas.put(pointThree, 0.0);
         }
      }

      else
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree, pointFour))
         {
            closestPointToOrigin.set(pointOne);
            retainPoints(pointOne);
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree, pointFour))
         {
            closestPointToOrigin.set(pointTwo);
            retainPoints(pointTwo);
         }
         else if (isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo, pointFour))
         {
            closestPointToOrigin.set(pointThree);
            retainPoints(pointThree);
         }
         else if (isInVoronoiRegionOfVertex(pointFour, pointOne, pointTwo, pointThree))
         {
            closestPointToOrigin.set(pointFour);
            retainPoints(pointFour);
         }

         else if (isInVornoiRegionOfEdge(pointOne, pointTwo, pointThree, pointFour))
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            retainPoints(pointOne, pointTwo);
         }
         else if (isInVornoiRegionOfEdge(pointOne, pointThree, pointTwo, pointFour))
         {
            projectOriginOntoEdge(pointOne, pointThree, closestPointToOrigin);
            retainPoints(pointOne, pointThree);
         }
         else if (isInVornoiRegionOfEdge(pointOne, pointFour, pointTwo, pointThree))
         {
            projectOriginOntoEdge(pointOne, pointFour, closestPointToOrigin);
            retainPoints(pointOne, pointFour);
         }
         else if (isInVornoiRegionOfEdge(pointTwo, pointThree, pointOne, pointFour))
         {
            projectOriginOntoEdge(pointTwo, pointThree, closestPointToOrigin);
            retainPoints(pointTwo, pointThree);
         }
         else if (isInVornoiRegionOfEdge(pointTwo, pointFour, pointOne, pointThree))
         {
            projectOriginOntoEdge(pointTwo, pointFour, closestPointToOrigin);
            retainPoints(pointTwo, pointFour);
         }
         else if (isInVornoiRegionOfEdge(pointThree, pointFour, pointOne, pointTwo))
         {
            projectOriginOntoEdge(pointThree, pointFour, closestPointToOrigin);
            retainPoints(pointThree, pointFour);
         }

         else if (isInVornoiRegionOfFace(pointOne, pointTwo, pointThree, pointFour))
         {
            projectOriginOntoFace(pointOne, pointTwo, pointThree, closestPointToOrigin);
            retainPoints(pointOne, pointTwo, pointThree);
         }
         else if (isInVornoiRegionOfFace(pointOne, pointTwo, pointFour, pointThree))
         {
            projectOriginOntoFace(pointOne, pointTwo, pointFour, closestPointToOrigin);
            retainPoints(pointOne, pointTwo, pointFour);
         }
         else if (isInVornoiRegionOfFace(pointOne, pointThree, pointFour, pointTwo))
         {
            projectOriginOntoFace(pointOne, pointThree, pointFour, closestPointToOrigin);
            retainPoints(pointOne, pointThree, pointFour);
         }
         else if (isInVornoiRegionOfFace(pointTwo, pointThree, pointFour, pointOne))
         {
            projectOriginOntoFace(pointTwo, pointThree, pointFour, closestPointToOrigin);
            retainPoints(pointTwo, pointThree, pointFour);
         }
         else
         {
            closestPointToOrigin.set(0.0, 0.0, 0.0);
            //TODO: Compute lambdas for the case of intersection

            lambdas.clear();
            lambdas.put(pointOne, 0.0);
            lambdas.put(pointTwo, 0.0);
            lambdas.put(pointThree, 0.0);
            lambdas.put(pointFour, 0.0);
         }
      }
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

      lambdas.clear();
      lambdas.put(pointToKeep, 1.0);
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

   private boolean isInVoronoiRegionOfVertex(Point3d pointToCheck, Point3d otherPoint)
   {
      tempVector1.set(pointToCheck);
      tempVector1.scale(-1.0);

      tempVector2.sub(otherPoint, pointToCheck);
      return (tempVector1.dot(tempVector2) <= 0.0);
   }

   private boolean isInVoronoiRegionOfVertex(Point3d pointToCheck, Point3d otherPoint1, Point3d otherPoint2)
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

   private boolean isInVoronoiRegionOfVertex(Point3d pointToCheck, Point3d otherPoint1, Point3d otherPoint2, Point3d otherPoint3)
   {
      tempVector1.set(pointToCheck);
      tempVector1.scale(-1.0);

      tempVector2.sub(otherPoint1, pointToCheck);
      if (!(tempVector1.dot(tempVector2) <= 0.0))
         return false;

      tempVector2.sub(otherPoint2, pointToCheck);
      if (!(tempVector1.dot(tempVector2) <= 0.0))
         return false;

      tempVector2.sub(otherPoint3, pointToCheck);
      if (!(tempVector1.dot(tempVector2) <= 0.0))
         return false;

      return true;
   }

   private final Vector3d tempVector1 = new Vector3d();
   private final Vector3d tempVector2 = new Vector3d();
   private final Vector3d tempVector3 = new Vector3d();
   private final Vector3d tempVector4 = new Vector3d();
   private final Vector3d tempNormalVector1 = new Vector3d();

   private boolean isInVornoiRegionOfEdge(Point3d edgePointOne, Point3d edgePointTwo, Point3d otherPoint)
   {
      tempVector1.sub(edgePointTwo, edgePointOne);
      tempVector2.sub(otherPoint, edgePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector2.cross(tempVector1, tempNormalVector1);
      tempVector3.set(edgePointOne);
      tempVector3.scale(-1.0);
      return (tempVector3.dot(tempVector2) >= 0.0);
   }

   private boolean isInVornoiRegionOfEdge(Point3d edgePointOne, Point3d edgePointTwo, Point3d otherPointOne, Point3d otherPointTwo)
   {
      tempVector1.sub(edgePointTwo, edgePointOne);
      tempVector2.sub(otherPointOne, edgePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector2.cross(tempVector1, tempNormalVector1);
      tempVector3.set(edgePointOne);
      tempVector3.scale(-1.0);
      if (!(tempVector3.dot(tempVector2) >= 0.0))
         return false;

      tempVector2.sub(otherPointTwo, edgePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector2.cross(tempVector1, tempNormalVector1);
      tempVector3.set(edgePointOne);
      tempVector3.scale(-1.0);
      if (!(tempVector3.dot(tempVector2) >= 0.0))
         return false;

      return true;
   }

   private boolean isInVornoiRegionOfFace(Point3d facePointOne, Point3d facePointTwo, Point3d facePointThree, Point3d otherPoint)
   {
      tempVector1.sub(facePointTwo, facePointOne);
      tempVector2.sub(facePointThree, facePointOne);

      tempNormalVector1.cross(tempVector1, tempVector2);

      tempVector1.set(facePointOne);
      tempVector1.scale(-1.0);
      double dot1 = tempVector1.dot(tempNormalVector1);

      tempVector3.sub(otherPoint, facePointOne);
      double dot2 = tempVector3.dot(tempNormalVector1);

      return (dot1 * dot2 < 0.0);
   }

   private void projectOriginOntoEdge(Point3d vertexOne, Point3d vertexTwo, Point3d projectionToPack)
   {
      tempVector1.set(vertexOne);
      tempVector1.scale(-1.0);
      tempVector2.sub(vertexTwo, vertexOne);

      double lambda = tempVector1.dot(tempVector2) / (tempVector2.dot(tempVector2));
      tempVector2.scale(lambda);

      projectionToPack.set(vertexOne);
      projectionToPack.add(tempVector2);

      double oneMinusLambda = 1.0 - lambda;

      lambdas.clear();
      lambdas.put(vertexOne, lambda);
      lambdas.put(vertexTwo, oneMinusLambda);

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

      tempNormalVector1.cross(tempVector1, tempVector2); //n
      double oneOver4ASquared = 1.0 / (tempNormalVector1.dot(tempNormalVector1));

      tempVector3.set(vertexOne);
      tempVector3.scale(-1.0); //w

      tempVector4.cross(tempVector1, tempVector3);
      double lambdaThree = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      tempVector4.cross(tempVector3, tempVector2);
      double lambdaTwo = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      double lambdaOne = 1.0 - lambdaTwo - lambdaThree;

      lambdas.clear();
      lambdas.put(vertexOne, lambdaOne);
      lambdas.put(vertexTwo, lambdaTwo);
      lambdas.put(vertexThree, lambdaThree);

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

}
