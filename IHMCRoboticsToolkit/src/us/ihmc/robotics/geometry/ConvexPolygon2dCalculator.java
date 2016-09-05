package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;

/**
 * This calculator class contains methods for computations with a ConvexPolygon2d such as
 * orthogonal projections and intersections.
 */
public class ConvexPolygon2dCalculator
{
   /**
    * Returns distance from the point to the boundary of this polygon. The return value
    * is positive if the point is inside and negative if it is outside.
    */
   public double getSignedDistance(Point2d point, ConvexPolygon2d polygon)
   {
      int numberOfVertices = polygon.getNumberOfVertices();
      if (numberOfVertices == 1)
         return -point.distance(polygon.getVertex(0));

      double closestDistance = Double.POSITIVE_INFINITY;
      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2d pointOne = polygon.getVertex(index);
         int nextIndex = index+1;
         if (nextIndex == numberOfVertices)
            nextIndex = 0;

         Point2d pointTwo = polygon.getVertex(nextIndex);

         double distance = computeDistanceToLineSegment(point, pointOne, pointTwo);
         if (distance < closestDistance)
            closestDistance = distance;
      }

      if (polygon.isPointInside(point))
         return closestDistance;
      return -closestDistance;
   }

   /**
    * Packs the closest vertex in the polygon to the given line.
    */
   public static boolean getClosestVertex(Line2d line, ConvexPolygon2d polygon, Point2d pointToPack)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      pointToPack.set(Double.NaN, Double.NaN);

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = polygon.getVertex(i);
         double distance = line.distance(vertex);
         if (distance < minDistance)
         {
            pointToPack.set(vertex);
            minDistance = distance;
         }
      }

      return !Double.isInfinite(minDistance);
   }

   /**
    * Packs the closest vertex to the polygon to the given point
    */
   public static boolean getClosestVertex(Point2d point, ConvexPolygon2d polygon, Point2d pointToPack)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      pointToPack.set(Double.NaN, Double.NaN);

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = polygon.getVertex(i);
         double distance = vertex.distance(point);
         if (distance < minDistance)
         {
            pointToPack.set(vertex);
            minDistance = distance;
         }
      }

      return !Double.isInfinite(minDistance);
   }


   // --- Methods that generate garbage ---
   public static Point2d getClosestVertexCopy(Line2d line, ConvexPolygon2d polygon)
   {
      Point2d ret = new Point2d();
      if (getClosestVertex(line, polygon, ret))
         return ret;
      return null;
   }

   public static Point2d getClosestVertexCopy(Point2d point, ConvexPolygon2d polygon)
   {
      Point2d ret = new Point2d();
      if (getClosestVertex(point, polygon, ret))
         return ret;
      return null;
   }

   // --- Internal methods ---
   private final LineSegment2d tempLineSegment = new LineSegment2d();
   private double computeDistanceToLineSegment(Point2d point, Point2d pointOne, Point2d pointTwo)
   {
      tempLineSegment.set(pointOne, pointTwo);
      return tempLineSegment.distance(point);
   }
}
