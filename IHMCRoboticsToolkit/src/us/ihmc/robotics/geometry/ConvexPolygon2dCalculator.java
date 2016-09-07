package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;

import us.ihmc.robotics.robotSide.RobotSide;

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
            Point2d edgeStart = polygon.getVertex(i);
            Point2d edgeEnd = polygon.getNextVertex(i);
            double distanceToEdgeLine = GeometryTools.distanceFromPointToLine(pointX, pointY, edgeStart.x, edgeStart.y, edgeEnd.x, edgeEnd.y);

            boolean pointOutside = canObserverSeeEdge(i, pointX, pointY, polygon);
            if (!pointOutside)
               distanceToEdgeLine = -distanceToEdgeLine;

            if (distanceToEdgeLine > epsilon)
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

   /**
    * Determines whether an observer can see the outside of the given edge. The edge index corresponds to
    * the vertex at the start of the edge when moving clockwise around the polygon.
    */
   public static boolean canObserverSeeEdge(int edgeIndex, double observerX, double observerY, ConvexPolygon2d polygon)
   {
      if (polygon.hasAtLeastTwoVertices())
      {
         Point2d vertexOne = polygon.getVertex(edgeIndex);
         Point2d vertexTwo = polygon.getNextVertex(edgeIndex);
         double edgeVectorX = vertexTwo.x - vertexOne.x;
         double edgeVectorY = vertexTwo.y - vertexOne.y;
         return Line2d.isPointOnSideOfLine(observerX, observerY, edgeVectorX, edgeVectorY, vertexOne.x, vertexOne.y, RobotSide.LEFT);
      }

      return false;
   }

   /**
    * Determines whether an observer can see the outside of the given edge. The edge index corresponds to
    * the vertex at the start of the edge when moving clockwise around the polygon.
    */
   public static boolean canObserverSeeEdge(int edgeIndex, Point2d observer, ConvexPolygon2d polygon)
   {
      return canObserverSeeEdge(edgeIndex, observer.x, observer.y, polygon);
   }

   /**
    * For an observer looking at the vertices corresponding to index1 and index2 this method will select the
    * index that corresponds to the vertex on the specified side.
    */
   public static int getVertexOnSide(int index1, int index2, RobotSide side, Point2d observer, ConvexPolygon2d polygon)
   {
      Point2d point1 = polygon.getVertex(index1);
      Point2d point2 = polygon.getVertex(index2);
      double observerToPoint1X = point1.getX() - observer.x;
      double observerToPoint1Y = point1.getY() - observer.y;
      double observerToPoint2X = point2.getX() - observer.x;
      double observerToPoint2Y = point2.getY() - observer.y;

      // Rotate the vector from observer to point 2 90 degree counter clockwise.
      double observerToPoint2PerpendicularX = - observerToPoint2Y;
      double observerToPoint2PerpendicularY = observerToPoint2X;

      // Assuming the observer is looking at point 1 the dot product will be positive if point 2 is on the right of point 1.
      double dotProduct = observerToPoint1X * observerToPoint2PerpendicularX + observerToPoint1Y * observerToPoint2PerpendicularY;

      dotProduct = side.negateIfLeftSide(dotProduct);
      if (dotProduct > 0.0)
         return index2;
      return index1;
   }

   /**
    * For an observer looking at the vertices corresponding to index1 and index2 this method will select the
    * index that corresponds to the vertex on the left side.
    */
   public static int getVertexOnLeft(int index1, int index2, Point2d observer, ConvexPolygon2d polygon)
   {
      return getVertexOnSide(index1, index2, RobotSide.LEFT, observer, polygon);
   }

   /**
    * For an observer looking at the vertices corresponding to index1 and index2 this method will select the
    * index that corresponds to the vertex on the right side.
    */
   public static int getVertexOnRight(int index1, int index2, Point2d observer, ConvexPolygon2d polygon)
   {
      return getVertexOnSide(index1, index2, RobotSide.RIGHT, observer, polygon);
   }

   /**
    * Returns the index in the middle of the range from firstIndex to secondIndex moving counter clockwise.
    * E.g. in a polygon with 6 vertices given indices 0 and 2 (in this order) the method will return the
    * middle of the range [0 5 4 3 2]: 4
    */
   public static int getMiddleIndexCounterClockwise(int firstIndex, int secondIndex, ConvexPolygon2d polygon)
   {
      int numberOfVertices = polygon.getNumberOfVertices();
      if (secondIndex >= firstIndex)
         return (secondIndex + (firstIndex + numberOfVertices  - secondIndex + 1) / 2) % numberOfVertices;
      else
         return (secondIndex + firstIndex + 1) / 2;
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
