package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

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
    * Returns the index of the closest vertex of the polygon to the given line.
    */
   public static int getClosestVertexIndex(Line2d line, ConvexPolygon2d polygon)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = polygon.getVertex(i);
         double distance = line.distance(vertex);
         if (distance < minDistance)
         {
            index = i;
            minDistance = distance;
         }
      }

      return index;
   }

   /**
    * Packs the closest vertex of the polygon to the given line.
    */
   public static boolean getClosestVertex(Line2d line, ConvexPolygon2d polygon, Point2d pointToPack)
   {
      int index = getClosestVertexIndex(line, polygon);
      if (index < 0)
         return false;
      pointToPack.set(polygon.getVertex(index));
      return true;
   }

   /**
    * Returns the index of the closest vertex of the polygon to the given point
    */
   public static int getClosestVertexIndex(Point2d point, ConvexPolygon2d polygon)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = polygon.getVertex(i);
         double distance = vertex.distance(point);
         if (distance < minDistance)
         {
            index = i;
            minDistance = distance;
         }
      }

      return index;
   }

   /**
    * Packs the closest vertex of the polygon to the given point
    */
   public static boolean getClosestVertex(Point2d point, ConvexPolygon2d polygon, Point2d pointToPack)
   {
      int index = getClosestVertexIndex(point, polygon);
      if (index < 0)
         return false;
      pointToPack.set(polygon.getVertex(index));
      return true;
   }

   /**
    * Determines if the point is inside the bounding box of the convex polygon.
    */
   public static boolean isPointInBoundingBox(double pointX, double pointY, double epsilon, ConvexPolygon2d polygon)
   {
      BoundingBox2d boundingBox = polygon.getBoundingBox();

      if (pointX < boundingBox.getMinPoint().getX() - epsilon)
         return false;
      if (pointY < boundingBox.getMinPoint().getY() - epsilon)
         return false;
      if (pointX > boundingBox.getMaxPoint().getX() + epsilon)
         return false;
      if (pointY > boundingBox.getMaxPoint().getY() + epsilon)
         return false;

      return true;
   }

   /**
    * Determines if the point is inside the bounding box of the convex polygon.
    */
   public static boolean isPointInBoundingBox(double pointX, double pointY, ConvexPolygon2d polygon)
   {
      return isPointInBoundingBox(pointX, pointY, 0.0, polygon);
   }

   /**
    * Determines if the pointToTest is inside the bounding box of the convex polygon.
    */
   public static boolean isPointInBoundingBox(Point2d pointToTest, double epsilon, ConvexPolygon2d polygon)
   {
      return isPointInBoundingBox(pointToTest.x, pointToTest.y, epsilon, polygon);
   }

   /**
    * Determines if the point is inside the bounding box of the convex polygon.
    */
   public static boolean isPointInBoundingBox(Point2d pointToTest, ConvexPolygon2d polygon)
   {
      return isPointInBoundingBox(pointToTest, 0.0, polygon);
   }

   /**
    * Determines if the point is inside the convex polygon.
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
    * Determines if the point is inside the convex polygon.
    */
   public static boolean isPointInside(double pointX, double pointY, ConvexPolygon2d polygon)
   {
      return isPointInside(pointX, pointY, 0.0, polygon);
   }

   /**
    * Determines if the pointToTest is inside the convex polygon.
    */
   public static boolean isPointInside(Point2d pointToTest, ConvexPolygon2d polygon)
   {
      return isPointInside(pointToTest, 0.0, polygon);
   }

   /**
    * Determines if the pointToTest is inside the convex polygon.
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
    * Determines if the polygonToTest is inside the convex polygon.
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
    * the vertex at the start of the edge when moving clockwise around the polygon. Will return false if the
    * observer is on the edge.
    */
   public static boolean canObserverSeeEdge(int edgeIndex, double observerX, double observerY, ConvexPolygon2d polygon)
   {
      Point2d vertexOne = polygon.getVertex(edgeIndex);
      Point2d vertexTwo = polygon.getNextVertex(edgeIndex);
      double edgeVectorX = vertexTwo.x - vertexOne.x;
      double edgeVectorY = vertexTwo.y - vertexOne.y;
      return Line2d.isPointOnSideOfLine(observerX, observerY, edgeVectorX, edgeVectorY, vertexOne.x, vertexOne.y, RobotSide.LEFT);
   }

   /**
    * Determines whether an observer can see the outside of the given edge. The edge index corresponds to
    * the vertex at the start of the edge when moving clockwise around the polygon. Will return false if the
    * observer is on the edge.
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
         return (secondIndex + (firstIndex + numberOfVertices - secondIndex + 1) / 2) % numberOfVertices;
      else
         return (secondIndex + firstIndex + 1) / 2;
   }

   /**
    * An observer looking at the polygon from the outside will see two vertices at the outside edges of the
    * polygon. This method packs the indices corresponding to these vertices. The vertex on the left from the
    * observer point of view will be the first vertex packed. The argument vertexIndices is expected to
    * have at least length two. If it is longer only the first two entries will be used.
    */
   public static boolean getLineOfSightVertexIndices(Point2d observer, int[] vertexIndicesToPack, ConvexPolygon2d polygon)
   {
      if (!polygon.hasAtLeastOneVertex())
         return false;

      if (polygon.hasExactlyOneVertex())
      {
         if (isPointInside(observer, polygon))
            return false;

         vertexIndicesToPack[0] = 0;
         vertexIndicesToPack[1] = 0;
         return true;
      }

      if (polygon.hasExactlyTwoVertices())
      {
         if (isPointInside(observer, polygon))
            return false;

         vertexIndicesToPack[0] = getVertexOnLeft(0, 1, observer, polygon);
         vertexIndicesToPack[1] = vertexIndicesToPack[0] == 0 ? 1 : 0;
         return true;
      }

      int numberOfVertices = polygon.getNumberOfVertices();
      boolean edgeVisible = ConvexPolygon2dCalculator.canObserverSeeEdge(numberOfVertices - 1, observer, polygon);
      int foundCorners = 0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         boolean nextEdgeVisible = ConvexPolygon2dCalculator.canObserverSeeEdge(i, observer, polygon);
         if (edgeVisible && !nextEdgeVisible)
         {
            vertexIndicesToPack[0] = i;
            foundCorners++;
         }
         if (!edgeVisible && nextEdgeVisible)
         {
            vertexIndicesToPack[1] = i;
            foundCorners++;
         }

         if (foundCorners == 2) break; // performance only
         edgeVisible = nextEdgeVisible;
      }

      if (foundCorners != 2)
         return false;

      return true;
   }

   /**
    * Computes the points of intersection between the ray and the polygon and packs them into pointToPack1 and
    * pointToPack2. If there is only one intersection it will be stored in pointToPack1. Returns the number of
    * intersections found. The ray is given as a Line2d, where the start point of the ray is the point used to
    * specify the line and the direction of the ray is given by the direction of the line.
    */
   public static int intersectionWithRay(Line2d ray, Point2d pointToPack1, Point2d pointToPack2, ConvexPolygon2d polygon)
   {
      int intersections = intersectionWithLine(ray, pointToPack1, pointToPack2, polygon);
      Point2d rayStart = ray.getPoint();
      Vector2d rayDirection = ray.getNormalizedVector();

      if (intersections == 2)
      {
         // check line intersection 2:
         double rayStartToPointX = pointToPack2.x - rayStart.x;
         double rayStartToPointY = pointToPack2.y - rayStart.y;
         double dotProduct = rayStartToPointX * rayDirection.x + rayStartToPointY * rayDirection.y;
         if (Math.signum(dotProduct) == -1.0)
            intersections--;
      }

      if (intersections >= 1)
      {
         // check line intersection 1:
         double rayStartToPointX = pointToPack1.x - rayStart.x;
         double rayStartToPointY = pointToPack1.y - rayStart.y;
         double dotProduct = rayStartToPointX * rayDirection.x + rayStartToPointY * rayDirection.y;
         if (Math.signum(dotProduct) == -1.0)
         {
            pointToPack1.set(pointToPack2);
            intersections--;
         }
      }

      return intersections;
   }

   /**
    * Computes the points of intersection between the line and the polygon and packs them into pointToPack1 and
    * pointToPack2. If there is only one intersection (line goes through a vertex) it will be stored in pointToPack1.
    * Returns the number of intersections found.
    */
   public static int intersectionWithLine(Line2d line, Point2d pointToPack1, Point2d pointToPack2, ConvexPolygon2d polygon)
   {
      if (polygon.hasExactlyOneVertex())
      {
         Point2d vertex = polygon.getVertex(0);
         if (line.isPointOnLine(vertex))
         {
            pointToPack1.set(vertex);
            return 1;
         }
         return 0;
      }

      if (polygon.hasExactlyTwoVertices())
      {
         Point2d vertex0 = polygon.getVertex(0);
         Point2d vertex1 = polygon.getVertex(1);
         if (line.isPointOnLine(vertex0) && line.isPointOnLine(vertex1))
         {
            pointToPack1.set(vertex0);
            pointToPack2.set(vertex1);
            return 2;
         }
      }

      int foundIntersections = 0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (doesLineIntersectEdge(line, i, polygon))
         {
            Point2d edgeStart = polygon.getVertex(i);
            Point2d edgeEnd = polygon.getNextVertex(i);
            double edgeVectorX = edgeEnd.x - edgeStart.x;
            double edgeVectorY = edgeEnd.y - edgeStart.y;
            Point2d lineStart = line.getPoint();
            Vector2d lineDirection = line.getNormalizedVector();
            double lambda = getIntersectionLambda(edgeStart.x, edgeStart.y, edgeVectorX, edgeVectorY, lineStart.x, lineStart.y, lineDirection.x,
                  lineDirection.y);

            if (foundIntersections == 0)
            {
               pointToPack1.set(edgeVectorX, edgeVectorY);
               pointToPack1.scale(lambda);
               pointToPack1.add(edgeStart);
            }
            else
            {
               pointToPack2.set(edgeVectorX, edgeVectorY);
               pointToPack2.scale(lambda);
               pointToPack2.add(edgeStart);
            }

            foundIntersections++;
            if (foundIntersections == 2) break; // performance only
         }
      }

      if (foundIntersections == 2 && pointToPack1.equals(pointToPack2))
         foundIntersections--;

      return foundIntersections;
   }

   /**
    * This finds the edges of the polygon that intersect the given line. Will pack the edges into edgeToPack1 and
    * edgeToPack2. Returns number of intersections found. The edged will be ordered according to their index.
    */
   public static int getIntersectingEdges(Line2d line, LineSegment2d edgeToPack1, LineSegment2d edgeToPack2, ConvexPolygon2d polygon)
   {
      int foundEdges = 0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (doesLineIntersectEdge(line, i, polygon))
         {
            if (foundEdges == 0)
               edgeToPack1.set(polygon.getVertex(i), polygon.getNextVertex(i));
            else
               edgeToPack2.set(polygon.getVertex(i), polygon.getNextVertex(i));
            foundEdges++;
         }

         if (foundEdges == 2) break; // performance only
      }

      return foundEdges;
   }

   /**
    * Checks if a line intersects the edge with the given index.
    */
   public static boolean doesLineIntersectEdge(Line2d line, int edgeIndex, ConvexPolygon2d polygon)
   {
      if (!polygon.hasAtLeastTwoVertices())
         return false;

      Point2d edgePointOne = polygon.getVertex(edgeIndex);
      Point2d edgePointTwo = polygon.getNextVertex(edgeIndex);

      double edgeVectorX = edgePointTwo.x - edgePointOne.x;
      double edgeVectorY = edgePointTwo.y - edgePointOne.y;
      double lambdaOne = getIntersectionLambda(edgePointOne.x, edgePointOne.y, edgeVectorX, edgeVectorY, line.getPoint().x, line.getPoint().y,
            line.getNormalizedVector().x, line.getNormalizedVector().y);
      if (Double.isNaN(lambdaOne) || lambdaOne < 0.0)
         return false;

      double edgeVectorInvX = edgePointOne.x - edgePointTwo.x;
      double edgeVectorInvY = edgePointOne.y - edgePointTwo.y;
      double lambdaTwo = getIntersectionLambda(edgePointTwo.x, edgePointTwo.y, edgeVectorInvX, edgeVectorInvY, line.getPoint().x, line.getPoint().y,
            line.getNormalizedVector().x, line.getNormalizedVector().y);
      if (lambdaTwo < 0.0)
         return false;

      return true;
   }

   /**
    * Method that intersects two lines. Returns lambda such that the intersection point is
    * intersection = point1 + lambda * direction1
    */
   public static double getIntersectionLambda(double point1X, double point1Y, double direction1X, double direction1Y, double point2X, double point2Y,
         double direction2X, double direction2Y)
   {
      if (direction2X == 0.0 && direction1X != 0.0)
         return (point2X - point1X) / direction1X;
      if (direction2Y == 0.0 && direction1Y != 0.0)
         return (point2Y - point1Y) / direction1Y;

      double denumerator = direction1X / direction2X - direction1Y / direction2Y;

      // check if lines parallel:
      if (Math.abs(denumerator) < 10E-10)
         return Double.NaN;

      double numerator = (point1Y - point2Y) / direction2Y - (point1X - point2X) / direction2X;
      return numerator / denumerator;
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

   public static int[] getLineOfSightVertexIndicesCopy(Point2d observer, ConvexPolygon2d polygon)
   {
      int[] ret = new int[2];
      if (getLineOfSightVertexIndices(observer, ret, polygon))
         return ret;
      return null;
   }

   public static Point2d[] getLineOfSightVerticesCopy(Point2d observer, ConvexPolygon2d polygon)
   {
      int[] indices = getLineOfSightVertexIndicesCopy(observer, polygon);
      if (indices == null)
         return null;

      Point2d point1 = new Point2d(polygon.getVertex(indices[0]));
      Point2d point2 = new Point2d(polygon.getVertex(indices[1]));

      if (indices[0] == indices[1])
         return new Point2d[] {point1};
      return new Point2d[] {point1, point2};
   }

   public static LineSegment2d[] getIntersectingEdgesCopy(Line2d line, ConvexPolygon2d polygon)
   {
      LineSegment2d edge1 = new LineSegment2d();
      LineSegment2d edge2 = new LineSegment2d();

      int edges = getIntersectingEdges(line, edge1, edge2, polygon);
      if (edges == 2)
         return new LineSegment2d[] {edge1, edge2};
      return null;
   }

   public static Point2d[] intersectionWithLineCopy(Line2d line, ConvexPolygon2d polygon)
   {
      Point2d point1 = new Point2d();
      Point2d point2 = new Point2d();

      int intersections = intersectionWithLine(line, point1, point2, polygon);
      if (intersections == 2)
         return new Point2d[] {point1, point2};
      if (intersections == 1)
         return new Point2d[] {point1};
      return null;
   }

   public static Point2d[] intersectionWithRayCopy(Line2d ray, ConvexPolygon2d polygon)
   {
      Point2d point1 = new Point2d();
      Point2d point2 = new Point2d();

      int intersections = intersectionWithRay(ray, point1, point2, polygon);
      if (intersections == 2)
         return new Point2d[] {point1, point2};
      if (intersections == 1)
         return new Point2d[] {point1};
      return null;
   }
}
