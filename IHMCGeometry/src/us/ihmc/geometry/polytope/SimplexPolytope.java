package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class SimplexPolytope
{
   private Point3d pointOne, pointTwo, pointThree, pointFour;

   private final Vector3d edge12 = new Vector3d();
   private final Vector3d edge13 = new Vector3d();
   private final Vector3d edge14 = new Vector3d();
   private final Vector3d edge23 = new Vector3d();
   private final Vector3d edge24 = new Vector3d();
   private final Vector3d edge34 = new Vector3d();

   private final Vector3d faceNormal123 = new Vector3d();
   private final Vector3d faceNormal124 = new Vector3d();
   private final Vector3d faceNormal234 = new Vector3d();

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
      else throw new RuntimeException("Only support SimplexPolytopes with at most 4 vertices");
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

   public void getClosestPointToOriginOnConvexHull(Point3d closestPointToOrigin)
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
         if (isInVoronoiRegionLineSegment(pointOne, pointTwo))
         {
            closestPointToOrigin.set(pointOne);
            return;
         }
         else if (isInVoronoiRegionLineSegment(pointTwo, pointOne))
         {
            closestPointToOrigin.set(pointTwo);
            return;
         }
         else
         {
            projectOriginToEdge(pointOne, pointTwo, closestPointToOrigin);
         }
      }


//
//
//      // Check Voronoi regions of the vertices:
//      boolean done = checkVoronoiRegionOfTheVertices();


   }

   private final Vector3d tempVector1 = new Vector3d();
   private final Vector3d tempVector2 = new Vector3d();

   private void projectOriginToEdge(Point3d vertexOne, Point3d vertexTwo, Point3d projectionToPack)
   {
      tempVector1.set(vertexOne);
      tempVector1.scale(-1.0);
      tempVector2.sub(vertexTwo, vertexOne);

      double dot = tempVector1.dot(tempVector2);

      tempVector2.normalize();
      tempVector2.scale(dot);

      projectionToPack.set(vertexOne);
      projectionToPack.add(tempVector2);
   }

   private boolean isInVoronoiRegionLineSegment(Point3d pointToCheck, Point3d otherPoint)
   {
      tempVector1.set(pointToCheck);
      tempVector1.scale(-1.0);
      tempVector2.sub(otherPoint, pointToCheck);
      return (tempVector1.dot(tempVector2) <= 0.0);
   }




//
//
//
//
//   private boolean checkVoronoiRegionOfTheVertices()
//   {
//      Point3d point1 = points.get(0);
//      Point3d point2 = points.get(1);
//      Point3d point3 = null;
//      Point3d point4 = null;
//
//      if (points.size() > 2)
//      {
//         point3 = points.get(2);
//      }
//
//      if (points.size() > 3)
//      {
//         point4 = points.get(3);
//      }
//
//      tempVector1.set(point1);
//      tempVector1.scale(-1.0);
//
//      if (!inVoronoiRegion(tempVector1, point2, point1)) return false;
//      tempVector2.sub(point2, point1);
//
//      double dot = tempVector1.dot(tempVector2);
//      if (dot > 0.0) return false;
//
//      if (point3 != null)
//      {
//         tempVector2.sub(point3, point1);
//
//         dot = tempVector1.dot(tempVector2);
//         if (dot > 0.0) return false;
//      }
//   }
//
//   private boolean inVoronoiRegion(Point3d pointToCheck, Point3d otherPoint1, Point3d otherPoint2, Point3d otherPoint3)
//   {
//
//   }
//
//   private boolean inVoronoiRegion(Vector3d pMinusQ, Point3d pointI, Point3d vertexToCheck)
//   {
//      tempVector2.sub(pointI, vertexToCheck);
//      double dot = pMinusQ.dot(tempVector2);
//      if (dot > 0.0) return true;
//
//      return false;
//   }

}
