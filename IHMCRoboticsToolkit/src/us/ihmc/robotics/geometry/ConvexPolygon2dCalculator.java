package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;

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
   public static double getSignedDistance(Point2d point, ConvexPolygon2d polygon)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      for (int index = 0; index < polygon.getNumberOfVertices(); index++)
      {
         Point2d pointOne = polygon.getVertex(index);
         Point2d pointTwo = polygon.getNextVertex(index);

         double distance = GeometryTools.distanceFromPointToLineSegment(point, pointOne, pointTwo);
         if (distance < closestDistance)
            closestDistance = distance;
      }

      if (isPointInside(point, polygon))
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

   /**
    * Determines if the pointToTest is inside the convex polygon (orientation method Nordbeck, Rystedt, 1967).
    */
   public static boolean isPointInside(double pointX, double pointY, double epsilon, ConvexPolygon2d polygon)
   {
      if (polygon.hasExactlyOneVertex())
      {
         Point2d vertex = polygon.getVertex(0);
         if (Math.abs(vertex.x - pointX) > epsilon)
            return false;
         if (Math.abs(vertex.y - pointY) > epsilon)
            return false;
         return true;
      }

      if (polygon.hasExactlyTwoVertices())
      {
         Point2d lineStart = polygon.getVertex(0);
         Point2d lineEnd = polygon.getVertex(1);
         double distance = GeometryTools.distanceFromPointToLineSegment(pointX, pointY, lineStart, lineEnd);
         if (distance > epsilon)
            return false;
         return true;
      }

      if (polygon.hasAtLeastTwoVertices())
      {
         // Determine whether the point is on the right side of each edge:
         for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         {
            double x0 = polygon.getVertex(i).getX();
            double y0 = polygon.getVertex(i).getY();

            double x1 = polygon.getNextVertex(i).getX();
            double y1 = polygon.getNextVertex(i).getY();

            if ((pointY - y0) * (x1 - x0) - (pointX - x0) * (y1 - y0) > epsilon)
               return false;
         }

         return true;
      }

      return false;
   }

   /**
    * Determines if the pointToTest is inside the convex polygon (orientation method Nordbeck, Rystedt, 1967).
    */
   public static boolean isPointInside(double pointX, double pointY, ConvexPolygon2d polygon)
   {
      return isPointInside(pointX, pointY, 0.0, polygon);
   }

   /**
    * Determines if the pointToTest is inside the convex polygon (orientation method Nordbeck, Rystedt, 1967).
    */
   public static boolean isPointInside(Point2d pointToTest, ConvexPolygon2d polygon)
   {
      return isPointInside(pointToTest, 0.0, polygon);
   }

   /**
    * Determines if the pointToTest is inside the convex polygon (orientation method Nordbeck, Rystedt, 1967).
    */
   public static boolean isPointInside(Point2d pointToTest, double epsilon, ConvexPolygon2d polygon)
   {
      return isPointInside(pointToTest.x, pointToTest.y, epsilon, polygon);
   }

   /**
    * Determines if the polygonToTest is inside the convex polygon.
    */
   public static boolean isPolygonInside(ConvexPolygon2d polygonToTest, double epsilon, ConvexPolygon2d polygon)
   {
      for (int i = 0; i < polygonToTest.getNumberOfVertices(); i++)
      {
         if (!isPointInside(polygonToTest.getVertex(i), epsilon, polygon))
            return false;
      }

      return true;
   }

   /**
    * Determines whether a polygon is inside the convex polygon.
    */
   public static boolean isPolygonInside(ConvexPolygon2d polygonToTest, ConvexPolygon2d polygon)
   {
      return isPolygonInside(polygonToTest, 0.0, polygon);
   }

   /**
    * Translates the given polygon.
    */
   public static void translatePolygon(Tuple2d translation, ConvexPolygon2d polygon)
   {
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = polygon.getVertex(i);
         vertex.add(translation);
      }
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

   public static ConvexPolygon2d translatePolygonCopy(Tuple2d translation, ConvexPolygon2d polygon)
   {
      ConvexPolygon2d ret = new ConvexPolygon2d(polygon);
      translatePolygon(translation, ret);
      return ret;
   }

}
