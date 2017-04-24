package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * This calculator class contains methods for computations with a ConvexPolygon2d such as
 * orthogonal projections and intersections.
 */
public class ConvexPolygon2dCalculator
{
   /**
    * Returns the index of the closest vertex of the polygon to the given line. If there
    * are multiple closest vertices (line parallel to an edge) this will return the smaller
    * index.
    */
   public static int getClosestVertexIndex(Line2d line, ConvexPolygon2d polygon)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygon.getVertex(i);
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
   public static boolean getClosestVertex(Line2d line, ConvexPolygon2d polygon, Point2DBasics pointToPack)
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
   public static int getClosestVertexIndex(Point2DReadOnly point, ConvexPolygon2d polygon)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygon.getVertex(i);
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
   public static boolean getClosestVertex(Point2DReadOnly point, ConvexPolygon2d polygon, Point2DBasics pointToPack)
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
      BoundingBox2D boundingBox = polygon.getBoundingBox();

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
   public static boolean isPointInBoundingBox(Point2DReadOnly pointToTest, double epsilon, ConvexPolygon2d polygon)
   {
      return isPointInBoundingBox(pointToTest.getX(), pointToTest.getY(), epsilon, polygon);
   }

   /**
    * Determines if the point is inside the bounding box of the convex polygon.
    */
   public static boolean isPointInBoundingBox(Point2DReadOnly pointToTest, ConvexPolygon2d polygon)
   {
      return isPointInBoundingBox(pointToTest, 0.0, polygon);
   }

   /**
    * Determines if the polygonToTest is inside the convex polygon.
    */
   public static boolean isPolygonInside(ConvexPolygon2d polygonToTest, double epsilon, ConvexPolygon2d polygon)
   {
      for (int i = 0; i < polygonToTest.getNumberOfVertices(); i++)
      {
         if (!polygon.isPointInside(polygonToTest.getVertex(i), epsilon))
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
   public static void translatePolygon(Tuple2DReadOnly translation, ConvexPolygon2d polygon)
   {
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2D vertex = polygon.getVertexUnsafe(i);
         vertex.add(translation);
      }

      polygon.updateBoundingBox();
      polygon.updateCentroidAndArea();
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
    * Packs a vector that is orthogonal to the given edge, facing towards the outside of the polygon
    */
   public static void getEdgeNormal(int edgeIndex, Vector2DBasics normalToPack, ConvexPolygon2d polygon)
   {
      Point2DReadOnly edgeStart = polygon.getVertex(edgeIndex);
      Point2DReadOnly edgeEnd = polygon.getNextVertex(edgeIndex);

      double edgeVectorX = edgeEnd.getX() - edgeStart.getX();
      double edgeVectorY = edgeEnd.getY() - edgeStart.getY();

      normalToPack.set(-edgeVectorY, edgeVectorX);
      normalToPack.normalize();
   }

   /**
    * This finds the edges of the polygon that intersect the given line. Will pack the edges into edgeToPack1 and
    * edgeToPack2. Returns number of intersections found. An edge parallel to the line can not intersect the edge.
    * If the line goes through a vertex but is not parallel to an edge adjacent to that vertex this method will
    * only pack the edge before the vertex, not both edges.
    */
   public static int getIntersectingEdges(Line2d line, LineSegment2d edgeToPack1, LineSegment2d edgeToPack2, ConvexPolygon2d polygon)
   {
      if (polygon.hasExactlyOneVertex())
         return 0;

      int foundEdges = 0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly startVertex = polygon.getVertex(i);
         Point2DReadOnly endVertex = polygon.getNextVertex(i);

         // edge is on the line
         if (line.isPointOnLine(startVertex) && line.isPointOnLine(endVertex))
         {
            if (polygon.hasExactlyTwoVertices())
               return 0;
            // set the edges to be the previous and the next edge
            edgeToPack1.set(polygon.getPreviousVertex(i), startVertex);
            edgeToPack2.set(endVertex, polygon.getNextVertex(polygon.getNextVertexIndex(i)));
            return 2;
         }

         if (line.isPointOnLine(startVertex))
            continue;

         if (doesLineIntersectEdge(line, i, polygon) || line.isPointOnLine(endVertex))
         {
            if (foundEdges == 0)
               edgeToPack1.set(startVertex, endVertex);
            else
               edgeToPack2.set(startVertex, endVertex);
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

      Point2DReadOnly edgeStart = polygon.getVertex(edgeIndex);
      Point2DReadOnly edgeEnd = polygon.getNextVertex(edgeIndex);

      double lineDirectionX = line.normalizedVector.getX();
      double lineDirectionY = line.normalizedVector.getY();
      double edgeDirectionX = edgeEnd.getX() - edgeStart.getX();
      double edgeDirectionY = edgeEnd.getY() - edgeStart.getY();

      if (EuclidGeometryTools.areVector2DsParallel(lineDirectionX, lineDirectionY, edgeDirectionX, edgeDirectionY, EuclidGeometryTools.ONE_TEN_MILLIONTH))
            return false;
      else
         return EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(line.point, line.normalizedVector, edgeStart, edgeEnd);
   }

   // --- Methods that generate garbage ---
   public static Point2D getClosestVertexCopy(Line2d line, ConvexPolygon2d polygon)
   {
      Point2D ret = new Point2D();
      if (getClosestVertex(line, polygon, ret))
         return ret;
      return null;
   }

   public static Point2D getClosestVertexCopy(Point2DReadOnly point, ConvexPolygon2d polygon)
   {
      Point2D ret = new Point2D();
      if (getClosestVertex(point, polygon, ret))
         return ret;
      return null;
   }

   public static ConvexPolygon2d translatePolygonCopy(Tuple2DReadOnly translation, ConvexPolygon2d polygon)
   {
      ConvexPolygon2d ret = new ConvexPolygon2d(polygon);
      translatePolygon(translation, ret);
      return ret;
   }

   public static LineSegment2d[] getIntersectingEdgesCopy(Line2d line, ConvexPolygon2d polygon)
   {
      LineSegment2d edge1 = new LineSegment2d();
      LineSegment2d edge2 = new LineSegment2d();

      int edges = getIntersectingEdges(line, edge1, edge2, polygon);
      if (edges == 2)
         return new LineSegment2d[] {edge1, edge2};
      if (edges == 1)
         return new LineSegment2d[] {edge1};
      return null;
   }
}
