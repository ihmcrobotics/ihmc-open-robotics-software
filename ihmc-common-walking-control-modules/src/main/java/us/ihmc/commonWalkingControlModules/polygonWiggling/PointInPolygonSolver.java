package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Solver to determine if a point is inside or outside a simple polygon. A simple polygon is a convex or concave polygon with no self intersections and no holes.
 * The solver performs a ray-cast from the query point along the x-axis. If a ray has an odd number of intersections, that indicates the point is inside the polygon.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm</a>
 * @see <a href="https://dl.acm.org/doi/10.1145/368637.368653</a>
 */
public class PointInPolygonSolver
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
}
