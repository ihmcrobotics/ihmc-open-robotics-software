package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class SimplexPolytope
{
   private Point3d pointOne, pointTwo, pointThree, pointFour;

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

   public void addPoint(Point3d point)
   {
      if (pointOne == null)
      {
         pointOne = point;
      }
      else if (pointTwo == null)
      {
         pointTwo = point;
      }
      else if (pointThree == null)
      {
         pointThree = point;
      }
      else if (pointFour == null)
      {
         pointFour = point;
      }
      else
         throw new RuntimeException("Only support SimplexPolytopes with at most 4 vertices");
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
      }

      else if (pointThree == null)
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo))
         {
            closestPointToOrigin.set(pointOne);
            pointTwo = null;
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne))
         {
            closestPointToOrigin.set(pointTwo);
            pointOne = pointTwo;
            pointTwo = null;
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
            pointTwo = pointThree = null;
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree))
         {
            closestPointToOrigin.set(pointTwo);
            pointOne = pointTwo;
            pointTwo = pointThree = null;
         }
         else if (isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo))
         {
            closestPointToOrigin.set(pointThree);
            pointOne = pointThree;
            pointTwo = pointThree = null;
         }

         else if (isInVornoiRegionOfEdge(pointOne, pointTwo, pointThree))
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            pointThree = null;
         }
         else if (isInVornoiRegionOfEdge(pointOne, pointThree, pointTwo))
         {
            projectOriginOntoEdge(pointOne, pointThree, closestPointToOrigin);
            pointTwo = pointThree;
            pointThree = null;
         }
         else if (isInVornoiRegionOfEdge(pointTwo, pointThree, pointOne))
         {
            projectOriginOntoEdge(pointTwo, pointThree, closestPointToOrigin);
            pointOne = pointTwo;
            pointTwo = pointThree;
            pointThree = null;
         }
         else
         {
            closestPointToOrigin.set(0.0, 0.0, 0.0);
         }
      }

      else
      {
         if (isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree, pointFour))
         {
            closestPointToOrigin.set(pointOne);
            pointTwo = pointThree = pointFour = null;
         }
         else if (isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree, pointFour))
         {
            closestPointToOrigin.set(pointTwo);
            pointOne = pointTwo;
            pointTwo = pointThree = pointFour = null;
         }
         else if (isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo, pointFour))
         {
            closestPointToOrigin.set(pointThree);
            pointOne = pointThree;
            pointTwo = pointThree = pointFour = null;
         }
         else if (isInVoronoiRegionOfVertex(pointFour, pointOne, pointTwo, pointThree))
         {
            closestPointToOrigin.set(pointFour);
            pointOne = pointFour;
            pointTwo = pointThree = pointFour = null;
         }

         else if (isInVornoiRegionOfEdge(pointOne, pointTwo, pointThree, pointFour))
         {
            projectOriginOntoEdge(pointOne, pointTwo, closestPointToOrigin);
            pointThree = pointFour = null;
         }
         else if (isInVornoiRegionOfEdge(pointOne, pointThree, pointTwo, pointFour))
         {
            projectOriginOntoEdge(pointOne, pointThree, closestPointToOrigin);
            pointTwo = pointThree;
            pointThree = pointFour = null;
         }
         else if (isInVornoiRegionOfEdge(pointOne, pointFour, pointTwo, pointThree))
         {
            projectOriginOntoEdge(pointOne, pointFour, closestPointToOrigin);
            pointTwo = pointFour;
            pointThree = pointFour = null;
         }
         else if (isInVornoiRegionOfEdge(pointTwo, pointThree, pointOne, pointFour))
         {
            projectOriginOntoEdge(pointTwo, pointThree, closestPointToOrigin);
            pointOne = pointTwo;
            pointTwo = pointThree;
            pointThree = pointFour = null;
         }
         else if (isInVornoiRegionOfEdge(pointTwo, pointFour, pointOne, pointThree))
         {
            projectOriginOntoEdge(pointTwo, pointFour, closestPointToOrigin);
            pointOne = pointTwo;
            pointTwo = pointFour;
            pointThree = pointFour = null;
         }
         else if (isInVornoiRegionOfEdge(pointThree, pointFour, pointOne, pointTwo))
         {
            projectOriginOntoEdge(pointThree, pointFour, closestPointToOrigin);
            pointOne = pointThree;
            pointTwo = pointFour;
            pointThree = pointFour = null;
         }

         else if (isInVornoiRegionOfFace(pointOne, pointTwo, pointThree, pointFour))
         {
            pointFour = null;
            projectOriginOntoFace(pointOne, pointTwo, pointThree, closestPointToOrigin);
         }
         else if (isInVornoiRegionOfFace(pointOne, pointTwo, pointFour, pointThree))
         {
            projectOriginOntoFace(pointOne, pointTwo, pointFour, closestPointToOrigin);
            pointThree = pointFour;
            pointFour = null;
         }
         else if (isInVornoiRegionOfFace(pointOne, pointThree, pointFour, pointTwo))
         {
            projectOriginOntoFace(pointOne, pointThree, pointFour, closestPointToOrigin);
            pointTwo = pointThree;
            pointThree = pointFour;
            pointFour = null;
         }
         else if (isInVornoiRegionOfFace(pointTwo, pointThree, pointFour, pointOne))
         {
            projectOriginOntoFace(pointTwo, pointThree, pointFour, closestPointToOrigin);
            pointOne = pointTwo;
            pointTwo = pointThree;
            pointThree = pointFour;
            pointFour = null;
         }
         else
         {
            closestPointToOrigin.set(0.0, 0.0, 0.0);
         }
      }
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

      tempVector2.normalize();
      double dot = tempVector1.dot(tempVector2);
      tempVector2.scale(dot);

      projectionToPack.set(vertexOne);
      projectionToPack.add(tempVector2);
   }

   private void projectOriginOntoFace(Point3d vertexOne, Point3d vertexTwo, Point3d vertexThree, Point3d closestPointToOrigin)
   {
      tempVector1.sub(vertexTwo, vertexOne);
      tempVector2.sub(vertexThree, vertexOne);

      tempNormalVector1.cross(tempVector1, tempVector2);
      tempNormalVector1.normalize();

      tempVector3.set(vertexOne);
      tempVector3.scale(-1.0);

      double dot = tempVector3.dot(tempNormalVector1);
      closestPointToOrigin.set(tempNormalVector1);
      closestPointToOrigin.scale(-dot);
   }

}
