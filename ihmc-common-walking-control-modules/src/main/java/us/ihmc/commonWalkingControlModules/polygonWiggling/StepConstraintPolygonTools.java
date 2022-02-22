package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

/**
 * Solver to determine if a point is inside or outside a simple polygon. A simple polygon is a convex or concave polygon with no self intersections and no holes.
 * The solver performs a ray-cast from the query point along the x-axis. If a ray has an odd number of intersections, that indicates the point is inside the polygon.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm</a>
 * @see <a href="https://dl.acm.org/doi/10.1145/368637.368653</a>
 */
public class StepConstraintPolygonTools
{
   public static boolean isPointInsidePolygon(Vertex2DSupplier polygon, Point2DReadOnly queryPoint)
   {
      if (polygon.getNumberOfVertices() < 3)
      {
         return false;
      }

      int intersections = 0;

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygon.getVertex(i);
         Point2DReadOnly nextVertex = polygon.getVertex((i + 1) % polygon.getNumberOfVertices());
         double lineDirectionX = 1.0;
         double lineDirectionY = 0.0;

         boolean intersects = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(queryPoint.getX(),
                                                                                           queryPoint.getY(),
                                                                                           lineDirectionX,
                                                                                           lineDirectionY,
                                                                                           vertex.getX(),
                                                                                           vertex.getY(),
                                                                                           nextVertex.getX(),
                                                                                           nextVertex.getY(),
                                                                                           null);

         if (intersects)
         {
            intersections++;
         }
      }

      boolean oddNumberOfIntersections = intersections % 2 == 1;
      return oddNumberOfIntersections;
   }

   public static boolean arePolygonsIntersecting(ConvexPolygon2DReadOnly polygonA, ConvexPolygon2DReadOnly polygonB)
   {
      for (int i = 0; i < polygonA.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vA1 = polygonA.getVertex(i);
         Point2DReadOnly vA2 = polygonA.getNextVertex(i);

         // in case one polygon is completely contained in the other
         if (polygonB.isPointInside(vA1))
         {
            return true;
         }

         for (int j = 0; j < polygonB.getNumberOfVertices(); j++)
         {
            Point2DReadOnly vB1 = polygonB.getVertex(j);
            Point2DReadOnly vB2 = polygonB.getNextVertex(j);

            if (polygonA.isPointInside(vB1))
            {
               return true;
            }

            double vA1x = vA1.getX();
            double vA1y = vA1.getY();
            double vA2x = vA2.getX();
            double vA2y = vA2.getY();
            double vB1x = vB1.getX();
            double vB1y = vB1.getY();
            double vB2x = vB2.getX();
            double vB2y = vB2.getY();
            boolean intersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(vA1x, vA1y, vA2x, vA2y, vB1x, vB1y, vB2x, vB2y, null);
            if (intersection)
            {
               return true;
            }
         }
      }

      return false;
   }

   /**
    * Written assuming that the polygons aren't intersecting. This is brute force and probably less efficient
    * than {@link ConvexPolygonTools#computeMinimumDistancePoints}, but that method was seen to give bad results for polygons
    * intersecting by epsilon (didn't seem to be picked up by the method's initial intersecion check, and probably throws off the algorithm).
    */
   public static double distanceBetweenPolygons(ConvexPolygon2D polygonA, ConvexPolygon2D polygonB)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < polygonA.getNumberOfVertices(); i++)
      {
         for (int j = 0; j < polygonB.getNumberOfVertices(); j++)
         {
            double vA1x = polygonA.getVertex(i).getX();
            double vA1y = polygonA.getVertex(i).getY();
            double vA2x = polygonA.getNextVertex(i).getX();
            double vA2y = polygonA.getNextVertex(i).getY();
            double vB1x = polygonB.getVertex(j).getX();
            double vB1y = polygonB.getVertex(j).getY();
            double vB2x = polygonB.getNextVertex(j).getX();
            double vB2y = polygonB.getNextVertex(j).getY();
            double distance = EuclidGeometryTools.closestPoint2DsBetweenTwoLineSegment2Ds(vA1x, vA1y, vA2x, vA2y, vB1x, vB1y, vB2x, vB2y, null, null);
            minDistance = Math.min(minDistance, distance);
         }
      }

      return minDistance;
   }

}
