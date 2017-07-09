package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * This calculator class contains methods for computations with a ConvexPolygon2d such as
 * orthogonal projections and intersections.
 */
public class ConvexPolygon2dCalculator
{
   /**
    * Determines if the polygonToTest is inside the convex polygon.
    */
   public static boolean isPolygonInside(ConvexPolygon2D polygonToTest, double epsilon, ConvexPolygon2D polygon)
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
   public static boolean isPolygonInside(ConvexPolygon2D polygonToTest, ConvexPolygon2D polygon)
   {
      return isPolygonInside(polygonToTest, 0.0, polygon);
   }

   /**
    * Returns the index in the middle of the range from firstIndex to secondIndex moving counter clockwise.
    * E.g. in a polygon with 6 vertices given indices 0 and 2 (in this order) the method will return the
    * middle of the range [0 5 4 3 2]: 4
    */
   public static int getMiddleIndexCounterClockwise(int firstIndex, int secondIndex, ConvexPolygon2D polygon)
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
   public static void getEdgeNormal(int edgeIndex, Vector2DBasics normalToPack, ConvexPolygon2D polygon)
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
   public static int getIntersectingEdges(Line2D line, LineSegment2D edgeToPack1, LineSegment2D edgeToPack2, ConvexPolygon2D polygon)
   {
      if (polygon.getNumberOfVertices() == 1)
         return 0;

      int foundEdges = 0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly startVertex = polygon.getVertex(i);
         Point2DReadOnly endVertex = polygon.getNextVertex(i);

         // edge is on the line
         if (line.isPointOnLine(startVertex) && line.isPointOnLine(endVertex))
         {
            if (polygon.getNumberOfVertices() == 2)
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
   public static boolean doesLineIntersectEdge(Line2D line, int edgeIndex, ConvexPolygon2D polygon)
   {
      if (polygon.getNumberOfVertices() < 2)
         return false;

      Point2DReadOnly edgeStart = polygon.getVertex(edgeIndex);
      Point2DReadOnly edgeEnd = polygon.getNextVertex(edgeIndex);

      double lineDirectionX = line.getDirectionX();
      double lineDirectionY = line.getDirectionY();
      double edgeDirectionX = edgeEnd.getX() - edgeStart.getX();
      double edgeDirectionY = edgeEnd.getY() - edgeStart.getY();

      if (EuclidGeometryTools.areVector2DsParallel(lineDirectionX, lineDirectionY, edgeDirectionX, edgeDirectionY, EuclidGeometryTools.ONE_TEN_MILLIONTH))
            return false;
      else
         return EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(line.getPoint(), line.getDirection(), edgeStart, edgeEnd);
   }

   // --- Methods that generate garbage ---
   public static LineSegment2D[] getIntersectingEdgesCopy(Line2D line, ConvexPolygon2D polygon)
   {
      LineSegment2D edge1 = new LineSegment2D();
      LineSegment2D edge2 = new LineSegment2D();

      int edges = getIntersectingEdges(line, edge1, edge2, polygon);
      if (edges == 2)
         return new LineSegment2D[] {edge1, edge2};
      if (edges == 1)
         return new LineSegment2D[] {edge1};
      return null;
   }
}
