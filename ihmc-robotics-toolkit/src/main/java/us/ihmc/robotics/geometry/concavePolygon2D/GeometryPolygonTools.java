package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.EuclidCoreMissingTools;

import java.util.Collection;
import java.util.List;

public class GeometryPolygonTools
{
   /**
    * Checks to see if the inner polygon is entirely inside the outer polygon.
    */
   public static boolean isPolygonInsideOtherPolygon(Vertex2DSupplier innerPolygon, ConcavePolygon2DReadOnly outerPolygon)
   {
      // if any of the points are outside, it fails.
      for (int i = 0; i < innerPolygon.getNumberOfVertices(); i++)
      {
         if (!outerPolygon.isPointInsideEpsilon(innerPolygon.getVertex(i), 1e-5))
            return false;
      }

      Point2D intersection = new Point2D();

      // if any of the points cross, it fails.
      for (int i = 0; i < innerPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = innerPolygon.getVertex(i);
         Point2DReadOnly nextVertex = innerPolygon.getVertex((i + 1) % innerPolygon.getNumberOfVertices());

         for (int j = 0; j < outerPolygon.getNumberOfVertices(); j++)
         {
            Point2DReadOnly otherVertex = outerPolygon.getVertex(j);
            Point2DReadOnly otherNextVertex = outerPolygon.getNextVertex(j);

            if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(vertex, nextVertex, otherVertex, otherNextVertex, intersection))
               return false;
         }
      }

      return true;
   }

   public static boolean doPolygonsIntersect(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB)
   {
      return doPolygonsIntersect(polygonA.getBoundingBox(), polygonB.getBoundingBox(), polygonA, polygonB);
   }

   public static boolean doPolygonsIntersect(BoundingBox2DReadOnly boundingBoxA,
                                             BoundingBox2DReadOnly boundingBoxB,
                                             Vertex2DSupplier polygonA,
                                             Vertex2DSupplier polygonB)
   {
      if (!boundingBoxA.intersectsInclusive(boundingBoxB))
         return false;

      return doPolygonsIntersect(polygonA, polygonB);
   }

   public static boolean doPolygonsIntersect(Vertex2DSupplier polygonA, Vertex2DSupplier polygonB)
   {
      return doPolygonsIntersectBruteForce(polygonA, polygonB);
   }

   public static boolean doPolygonsIntersectBruteForce(Vertex2DSupplier polygonA, Vertex2DSupplier polygonB)
   {
      for (int i = 0; i < polygonA.getNumberOfVertices(); i++)
      {
         Point2DReadOnly startVertex = polygonA.getVertex(i);
         Point2DReadOnly endVertex = polygonA.getVertex(EuclidGeometryPolygonTools.next(i, polygonA.getNumberOfVertices()));

         if (doesLineSegment2DIntersectPolygon(startVertex, endVertex, polygonB))
            return true;
      }

      return false;
   }

   public static boolean doesLineSegment2DIntersectPolygon(Point2DReadOnly segmentStart, Point2DReadOnly segmentEnd, Vertex2DSupplier polygon)
   {
      Point2D intersection = new Point2D();
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly startVertex = polygon.getVertex(i);
         Point2DReadOnly endVertex = polygon.getVertex(EuclidGeometryPolygonTools.next(i, polygon.getNumberOfVertices()));

         // Don't count it if they're collinear.
         if (EuclidGeometryTools.areLine2DsCollinear(segmentStart, segmentEnd, startVertex, endVertex, 1e-5, 1e-4))
            continue;

         if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(segmentStart, segmentEnd, startVertex, endVertex, intersection))
            return true;
      }

      return false;
  }

   public static boolean isClockwiseOrdered3DZUp(List<? extends Point3DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      checkNumberOfVertices3D(concaveHullVertices, numberOfVertices);

      double sumOfAngles = 0.0;

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         int previousVertexIndex = EuclidGeometryPolygonTools.previous(vertexIndex, numberOfVertices);
         int nextVertexIndex = EuclidGeometryPolygonTools.next(vertexIndex, numberOfVertices);

         Point3DReadOnly previousVertex = concaveHullVertices.get(previousVertexIndex);
         Point3DReadOnly vertex = concaveHullVertices.get(vertexIndex);
         Point3DReadOnly nextVertex = concaveHullVertices.get(nextVertexIndex);

         double firstVectorX = vertex.getX() - previousVertex.getX();
         double firstVectorY = vertex.getY() - previousVertex.getY();
         double secondVectorX = nextVertex.getX() - vertex.getX();
         double secondVectorY = nextVertex.getY() - vertex.getY();
         sumOfAngles += angle(firstVectorX, firstVectorY, secondVectorX, secondVectorY);
      }

      return sumOfAngles <= 0.0;
   }

   public static boolean isClockwiseOrdered(List<? extends Point2DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      checkNumberOfVertices(concaveHullVertices, numberOfVertices);

      double sumOfAngles = 0.0;

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         int previousVertexIndex = EuclidGeometryPolygonTools.previous(vertexIndex, numberOfVertices);
         int nextVertexIndex = EuclidGeometryPolygonTools.next(vertexIndex, numberOfVertices);

         Point2DReadOnly previousVertex = concaveHullVertices.get(previousVertexIndex);
         Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
         Point2DReadOnly nextVertex = concaveHullVertices.get(nextVertexIndex);

         double firstVectorX = vertex.getX() - previousVertex.getX();
         double firstVectorY = vertex.getY() - previousVertex.getY();
         double secondVectorX = nextVertex.getX() - vertex.getX();
         double secondVectorY = nextVertex.getY() - vertex.getY();
         sumOfAngles += angle(firstVectorX, firstVectorY, secondVectorX, secondVectorY);
      }

      return sumOfAngles <= 0.0;
   }

   /**
    * Solver to determine if a point is inside or outside a simple polygon. A simple polygon is a convex or concave polygon with no self intersections and no
    * holes.
    * The solver performs a ray-cast from the query point along the x-axis. If a ray has an odd number of intersections, that indicates the point is inside the
    * polygon.
    *
    * @see <a href="https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm</a>
    * @see <a href="https://dl.acm.org/doi/10.1145/368637.368653</a>
    */
   public static boolean isPoint2DInsideSimplePolygon2D(Point2DReadOnly queryPoint, List<? extends Point2DReadOnly> polygon, int numberOfVertices)
   {
      return isPoint2DInsideSimplePolygon2D(queryPoint.getX(), queryPoint.getY(), polygon, numberOfVertices);
   }

   /**
    * Solver to determine if a point is inside or outside a simple polygon. A simple polygon is a convex or concave polygon with no self intersections and no
    * holes.
    * The solver performs a ray-cast from the query point along the x-axis. If a ray has an odd number of intersections, that indicates the point is inside the
    * polygon.
    *
    * @see <a href="https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm</a>
    * @see <a href="https://dl.acm.org/doi/10.1145/368637.368653</a>
    */
   public static boolean isPoint2DInsideSimplePolygon2D(double pointX, double pointY, List<? extends Point2DReadOnly> polygon, int numberOfVertices)
   {
      return isPoint2DInsideSimplePolygon2D(pointX, pointY, polygon, numberOfVertices, null, 1e-7);
   }

   public static boolean isPoint2DInsideSimplePolygon2D(Point2DReadOnly queryPoint,
                                                        List<? extends Point2DReadOnly> polygon,
                                                        int numberOfVertices,
                                                        double epsilon)
   {
      return isPoint2DInsideSimplePolygon2D(queryPoint.getX(), queryPoint.getY(), polygon, numberOfVertices, epsilon);
   }

   public static boolean isPoint2DInsideSimplePolygon2D(double pointX,
                                                        double pointY,
                                                        List<? extends Point2DReadOnly> polygon,
                                                        int numberOfVertices,
                                                        double epsilon)
   {
      return isPoint2DInsideSimplePolygon2D(pointX, pointY, polygon, numberOfVertices, null, epsilon);
   }

   public static boolean isPoint2DInsideSimplePolygon2D(double pointX,
                                                        double pointY,
                                                        List<? extends Point2DReadOnly> polygon,
                                                        int numberOfVertices,
                                                        Point2DBasics intersectionToPack,
                                                        double epsilon)
   {
      // check if point is on perimeter
      if (isPoint2DOnPerimeterOfSimplePolygon2D(pointX, pointY, polygon, numberOfVertices, epsilon))
         return true;

      return isPoint2DStrictlyInsideSimplePolygon2D(pointX, pointY, polygon, numberOfVertices, intersectionToPack, false, epsilon);
   }

   public static boolean isPoint2DOnPerimeterOfSimplePolygon2D(Point2DReadOnly point, List<? extends Point2DReadOnly> polygon, int numberOfVertices, double epsilon)
   {
      return isPoint2DOnPerimeterOfSimplePolygon2D(point.getX(), point.getY(), polygon, numberOfVertices, epsilon);
   }

   public static boolean isPoint2DOnPerimeterOfSimplePolygon2D(double pointX,
                                                               double pointY,
                                                               List<? extends Point2DReadOnly> polygon,
                                                               int numberOfVertices,
                                                               double epsilon)
   {
      checkNumberOfVertices(polygon, numberOfVertices);
      double epsilonSquared = MathTools.square(epsilon);

      // check if point is on perimeter
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex = polygon.get(i);
         Point2DReadOnly nextVertex = polygon.get((i + 1) % numberOfVertices);

         if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(pointX, pointY, vertex, nextVertex) < epsilonSquared)
            return true;
      }

      return false;
   }

   public static boolean isPoint2DStrictlyInsideSimplePolygon2D(Point2DReadOnly point,
                                                                List<? extends Point2DReadOnly> polygon,
                                                                int numberOfVertices,
                                                                Point2DBasics intersectionToPack,
                                                                double epsilon)
   {
      return isPoint2DStrictlyInsideSimplePolygon2D(point, polygon, numberOfVertices, intersectionToPack, true, epsilon);
   }

   public static boolean isPoint2DStrictlyInsideSimplePolygon2D(double pointX,
                                                                double pointY,
                                                                List<? extends Point2DReadOnly> polygon,
                                                                int numberOfVertices,
                                                                Point2DBasics intersectionToPack,
                                                                double epsilon)
   {
      return isPoint2DStrictlyInsideSimplePolygon2D(pointX, pointY, polygon, numberOfVertices, intersectionToPack, true, epsilon);
   }

   public static boolean isPoint2DStrictlyInsideSimplePolygon2D(Point2DReadOnly point,
                                                                List<? extends Point2DReadOnly> polygon,
                                                                int numberOfVertices,
                                                                Point2DBasics intersectionToPack,
                                                                boolean checkPerimeter,
                                                                double epsilon)
   {
      return isPoint2DStrictlyInsideSimplePolygon2D(point.getX(), point.getY(), polygon, numberOfVertices, intersectionToPack, checkPerimeter, epsilon);
   }

   public static boolean isPoint2DStrictlyInsideSimplePolygon2D(double pointX,
                                                                double pointY,
                                                                List<? extends Point2DReadOnly> polygon,
                                                                int numberOfVertices,
                                                                Point2DBasics intersectionToPack,
                                                                boolean checkPerimeter,
                                                                double epsilon)
   {
      if (checkPerimeter && isPoint2DOnPerimeterOfSimplePolygon2D(pointX, pointY, polygon, numberOfVertices, epsilon))
         return false;

      checkNumberOfVertices(polygon, numberOfVertices);

      if (numberOfVertices < 3)
         return false;

      double lineDirectionX = 1.0;
      double lineDirectionY = 0.0;

      int intersectionsPositive = getNumberOfIntersections(pointX,
                                                           pointY,
                                                           lineDirectionX,
                                                           lineDirectionY,
                                                           polygon,
                                                           numberOfVertices,
                                                           intersectionToPack,
                                                           epsilon);
      int intersectionsNegative = getNumberOfIntersections(pointX,
                                                           pointY,
                                                           -lineDirectionX,
                                                           -lineDirectionY,
                                                           polygon,
                                                           numberOfVertices,
                                                           intersectionToPack,
                                                           epsilon);

      if (intersectionsNegative == 0 || intersectionsPositive == 0)
         return false;

      if (intersectionsPositive % 2 == 0 && intersectionsNegative % 2 == 0)
         return false;

      boolean evenNumberOfIntersections = (intersectionsPositive + intersectionsNegative) % 2 == 0;
      return evenNumberOfIntersections;
   }


   private static int getNumberOfIntersections(double pointX,
                                               double pointY,
                                               double lineDirectionX,
                                               double lineDirectionY,
                                               List<? extends Point2DReadOnly> polygon,
                                               int numberOfVertices,
                                               Point2DBasics intersectionToPack,
                                               double epsilon)
   {
      int intersections = 0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         int prevIdx = EuclidGeometryPolygonTools.previous(i, numberOfVertices);
         int nextIdx = EuclidGeometryPolygonTools.next(i, numberOfVertices);
         int nextNextIdx = EuclidGeometryPolygonTools.next(nextIdx, numberOfVertices);
         Point2DReadOnly previousVertex = polygon.get(prevIdx);
         Point2DReadOnly vertex = polygon.get(i);
         Point2DReadOnly nextVertex = polygon.get(nextIdx);
         Point2DReadOnly nextNextVertex = polygon.get(nextNextIdx);

         boolean linesAreCollinear = EuclidGeometryTools.areLine2DsCollinear(pointX,
                                                                             pointY,
                                                                             lineDirectionX,
                                                                             lineDirectionY,
                                                                             vertex.getX(),
                                                                             vertex.getY(),
                                                                             nextVertex.getX() - vertex.getX(),
                                                                             nextVertex.getY() - vertex.getY(),
                                                                             1e-7,
                                                                             epsilon);

         boolean isSegmentATurnAround =
               EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(previousVertex, vertex, nextVertex) == EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(
                     nextNextVertex,
                     vertex,
                     nextVertex);

         // if the lines are collinear, then we don't count this one, as it intersects with both the start and end of the next one.
         if (linesAreCollinear && !isSegmentATurnAround)
            continue;

         // skip the first point. That intersection should get captured on the loop closure.
         if (EuclidGeometryTools.distanceFromPoint2DToRay2D(vertex.getX(), vertex.getY(), pointX, pointY, lineDirectionX, lineDirectionY) < epsilon)
         {

            if (linesAreCollinear)
               intersections++;
            continue;
         }

         boolean intersects = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(pointX,
                                                                                           pointY,
                                                                                           lineDirectionX,
                                                                                           lineDirectionY,
                                                                                           vertex.getX(),
                                                                                           vertex.getY(),
                                                                                           nextVertex.getX(),
                                                                                           nextVertex.getY(),
                                                                                           intersectionToPack);

         if (intersects)
         {
            intersections++;
         }
      }

      return intersections;
   }

   public static boolean isSimplePolygon(List<? extends Point2DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      // TODO implement some other versions of this algorithm that are faster (see https://www.webcitation.org/6ahkPQIsN)
      return isSimplePolygonBruteForce(concaveHullVertices, numberOfVertices);
   }

   public static boolean isSimplePolygonBruteForce(List<? extends Point2DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      for (int index = 0; index < numberOfVertices; index++)
      {
         int previousIndex = EuclidGeometryPolygonTools.previous(index, numberOfVertices);

         Point2DReadOnly segmentStart = concaveHullVertices.get(previousIndex);
         Point2DReadOnly segmentEnd = concaveHullVertices.get(index);

         int nextSegmentStart = EuclidGeometryPolygonTools.next(index, numberOfVertices);
         int nextSegmentEnd = EuclidGeometryPolygonTools.next(nextSegmentStart, numberOfVertices);
         while (nextSegmentEnd != previousIndex)
         {
            if (EuclidGeometryTools.doLineSegment2DsIntersect(segmentStart,
                                                              segmentEnd,
                                                              concaveHullVertices.get(nextSegmentStart),
                                                              concaveHullVertices.get(nextSegmentEnd)))
            {
               return false;
            }

            nextSegmentStart = nextSegmentEnd;
            nextSegmentEnd = EuclidGeometryPolygonTools.next(nextSegmentStart, numberOfVertices);
         }
      }

      return true;
   }

   private static double angle(Vector2DReadOnly firstVector, Vector2DReadOnly secondVector)
   {
      return angle(firstVector.getX(), firstVector.getY(), secondVector.getX(), secondVector.getY());
   }

   private static double angle(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY)
   {
      // The sign of the angle comes from the cross product
      double crossProduct = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      // the magnitude of the angle comes from the dot product
      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY;

      return EuclidCoreTools.atan2(crossProduct, dotProduct);
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }

   private static void checkNumberOfVertices3D(List<? extends Point3DReadOnly> convexPolygon, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon.size() + "].");

   }
}
