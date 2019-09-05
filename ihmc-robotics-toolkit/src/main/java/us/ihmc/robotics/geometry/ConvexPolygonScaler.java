package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Bound;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConvexPolygonScaler
{
   private final LineSegment2D polygonAsLineSegment = new LineSegment2D();
   private final Point2D newVertex0 = new Point2D();
   private final Point2D newVertex1 = new Point2D();
   private final ArrayList<Line2D> rays = new ArrayList<Line2D>();

   private final Line2D edgeOnQ = new Line2D();
   private final Vector2D vectorPerpendicularToEdgeOnQ = new Vector2D();
   private final Line2D linePerpendicularToEdgeOnQ = new Line2D();
   private final Point2D referencePoint = new Point2D();
   private final Vector2D normalizedVector = new Vector2D();
   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();

   private final ArrayList<Line2D> edgePool = new ArrayList<Line2D>();

   private final ConvexPolygonConstructorFromInteriorOfRays convexPolygonConstructorFromInteriorOfRays = new ConvexPolygonConstructorFromInteriorOfRays();

   public ConvexPolygonScaler()
   {
      for (int i = 0; i < 16; i++)
      {
         edgePool.add(new Line2D());
      }
   }

   private Line2D getARay(int index)
   {
      if (edgePool.size() <= index)
      {
         for (int i=0; i<index - edgePool.size() + 1; i++)
         {
            edgePool.add(new Line2D());
         }
      }

      return edgePool.get(index);
   }

   /**
    * Grows or shrinks the size of the polygon, If distance is positive it shrinks the polygon in by the distance in meters,
    * If the distance is negative it grows the polygon. If polygonQ is a line and the distance is negative, a 6 point polygon is returned around the line. If
    * polygonQ is a point, a square is returned around the point. polygonQ is not changed.
    */
   public boolean scaleConvexPolygon(ConvexPolygon2DReadOnly polygonQ, double distance, ConvexPolygon2DBasics polygonToPack)
   {
      return scaleConvexPolygon(polygonQ, distance, polygonToPack, -1);
   }


   /**
    * Grows or shrinks the size of the polygon, If distance is positive it shrinks the polygon in by the distance in meters,
    * If the distance is negative it grows the polygon. If polygonQ is a line and the distance is negative, a 6 point polygon is returned around the line. If
    * polygonQ is a point, a square is returned around the point. polygonQ is not changed.
    */
   public boolean scaleConvexPolygon(ConvexPolygon2DReadOnly polygonQ, double distance, ConvexPolygon2DBasics polygonToPack, int... vertexStartIndicesToNotScale)
   {
      if (Math.abs(distance) < 1.0e-10)
      {
         polygonToPack.set(polygonQ);
         return true;
      }

      if (polygonQ.getNumberOfVertices() == 2)
      {
         Point2DReadOnly vertex0 = polygonQ.getVertex(0);
         Point2DReadOnly vertex1 = polygonQ.getVertex(1);
         polygonAsLineSegment.set(vertex0, vertex1);

         if(distance < 0.0)
         {
            polygonToPack.clear();
            polygonAsLineSegment.direction(true, normalizedVector);
            normalizedVector.scale(-distance);
            polygonToPack.addVertex(vertex0.getX() - normalizedVector.getX(), vertex0.getY() - normalizedVector.getY());
            polygonToPack.addVertex(vertex1.getX() + normalizedVector.getX(), vertex1.getY() + normalizedVector.getY());

            polygonAsLineSegment.perpendicular(true, normalizedVector);
            normalizedVector.scale(distance);

            polygonToPack.addVertex(vertex0.getX() + normalizedVector.getX(), vertex0.getY() + normalizedVector.getY());
            polygonToPack.addVertex(vertex0.getX() - normalizedVector.getX(), vertex0.getY() - normalizedVector.getY());
            polygonToPack.addVertex(vertex1.getX() + normalizedVector.getX(), vertex1.getY() + normalizedVector.getY());
            polygonToPack.addVertex(vertex1.getX() - normalizedVector.getX(), vertex1.getY() - normalizedVector.getY());
            polygonToPack.update();
            return true;
         }

         if (vertex0.distance(vertex1) < 2.0 * distance)
         {
            Point2D midPoint = new Point2D(vertex0);
            midPoint.add(vertex1);
            midPoint.scale(0.5);

            polygonToPack.clear();
            polygonToPack.addVertex(midPoint);
            polygonToPack.update();
            return false;
         }

         double percentageAlongSegment = distance / polygonAsLineSegment.length();
         polygonAsLineSegment.pointBetweenEndpointsGivenPercentage(percentageAlongSegment, newVertex0);
         polygonAsLineSegment.pointBetweenEndpointsGivenPercentage(1 - percentageAlongSegment, newVertex1);

         polygonToPack.clear();
         polygonToPack.addVertex(newVertex0);
         polygonToPack.addVertex(newVertex1);
         polygonToPack.update();

         return true;
      }

      if (polygonQ.getNumberOfVertices() == 1)
      {
         if(distance < 0.0)
         {
            Point2DReadOnly vertex0 = polygonQ.getVertex(0);
            polygonToPack.addVertex(vertex0.getX() + distance, vertex0.getY() + distance);
            polygonToPack.addVertex(vertex0.getX() + distance, vertex0.getY() - distance);
            polygonToPack.addVertex(vertex0.getX() - distance, vertex0.getY() + distance);
            polygonToPack.addVertex(vertex0.getX() - distance, vertex0.getY() - distance);
            polygonToPack.update();
            return true;
         }
         polygonToPack.set(polygonQ);
         return false;
      }

      rays.clear();

      int leftMostIndexOnPolygonQ = EuclidGeometryPolygonTools.findVertexIndex(polygonQ, true, Bound.MIN, Bound.MIN);
      Point2DReadOnly vertexQ = polygonQ.getVertex(leftMostIndexOnPolygonQ);
      int vertexQIndex = leftMostIndexOnPolygonQ;
      int nextVertexQIndex = polygonQ.getNextVertexIndex(leftMostIndexOnPolygonQ);
      Point2DReadOnly nextVertexQ = polygonQ.getVertex(nextVertexQIndex);

      for (int i = 0; i < polygonQ.getNumberOfVertices(); i++)
      {
         edgeOnQ.set(vertexQ, nextVertexQ);
         normalizedVector.set(edgeOnQ.getDirection());

         if (!containsIndex(vertexQIndex, vertexStartIndicesToNotScale))
         {
            edgeOnQ.perpendicularVector(vectorPerpendicularToEdgeOnQ);
            vectorPerpendicularToEdgeOnQ.negate();
            linePerpendicularToEdgeOnQ.set(vertexQ, vectorPerpendicularToEdgeOnQ);
            linePerpendicularToEdgeOnQ.pointOnLineGivenParameter(distance, referencePoint);
         }
         else
         {
            referencePoint.set(vertexQ);
         }


         Line2D newEdge = getARay(rays.size());
         newEdge.set(referencePoint, normalizedVector);
         rays.add(newEdge);

         vertexQIndex = nextVertexQIndex;
         nextVertexQIndex = polygonQ.getNextVertexIndex(nextVertexQIndex);

         vertexQ = nextVertexQ;
         nextVertexQ = polygonQ.getVertex(nextVertexQIndex);
      }


      boolean foundSolution = convexPolygonConstructorFromInteriorOfRays.constructFromInteriorOfRays(rays, polygonToPack);
      if (!foundSolution)
      {
         polygonToPack.clear();
         polygonToPack.addVertex(polygonQ.getCentroid());
         polygonToPack.update();
      }

      return foundSolution;
   }

   private static boolean containsIndex(int indexToCheck, int... indicesToCheck)
   {
      if (indexToCheck < 0)
         return false;

      for (int i = 0; i < indicesToCheck.length; i++)
      {
         if (indexToCheck == indicesToCheck[i])
            return true;
      }

      return false;
   }

   private final Vector2D vectorToInteriorPolygonVertex = new Vector2D();

   /**
    * This function computes the inscribed polygon that represents the constraint on the centroid location of an interior polygon that must
    * remain inside an exterior polygon. The distance inside that the interior polygon can achieve is set by {@param distanceInside}, where positive
    * represents an interior offset, and negative represents an exterior offset.
    */
   public boolean scaleConvexPolygonToContainInteriorPolygon(ConvexPolygon2D exteriorPolygon, ConvexPolygon2D interiorPolygon, double distanceInside,
                                                             ConvexPolygon2D scaledPolygonToPack)
   {
      if (Math.abs(distanceInside) < 1.0e-10 && interiorPolygon.getArea() <= 1.0e-10)
      {
         scaledPolygonToPack.set(exteriorPolygon);
         return true;
      }

      if (exteriorPolygon.getNumberOfVertices() == 2)
      {

         Point2DReadOnly exteriorVertex1 = exteriorPolygon.getVertex(0);
         Point2DReadOnly exteriorVertex2 = exteriorPolygon.getVertex(1);
         edgeOnQ.set(exteriorVertex1, exteriorVertex2);
         polygonAsLineSegment.set(exteriorVertex1, exteriorVertex2);

         // first, expanding the polygon line into a six pointed polygon, then shrinking this polygon to contain the interior polygon
         if(distanceInside < 0.0)
         {
            scaleConvexPolygon(exteriorPolygon, distanceInside, tempPolygon);
            return scaleConvexPolygonToContainInteriorPolygon(tempPolygon, interiorPolygon, 0.0, scaledPolygonToPack);
         }

         double extraDistanceToPoint1 = 0.0;

         int leftMostIndexOnInteriorPolygon = EuclidGeometryPolygonTools.findVertexIndex(interiorPolygon, true, Bound.MIN, Bound.MIN);
         Point2DReadOnly interiorVertex = interiorPolygon.getVertex(leftMostIndexOnInteriorPolygon);
         int nextInteriorVertexIndex = interiorPolygon.getNextVertexIndex(leftMostIndexOnInteriorPolygon);
         Point2DReadOnly nextInteriorVertex = interiorPolygon.getVertex(nextInteriorVertexIndex);

         for (int j = 0; j < interiorPolygon.getNumberOfVertices(); j++)
         {
            vectorToInteriorPolygonVertex.set(interiorVertex);
            double projectedDistanceToPoint = edgeOnQ.getDirectionX() * vectorToInteriorPolygonVertex.getX() +
                  edgeOnQ.getDirectionY() * vectorToInteriorPolygonVertex.getY();

            extraDistanceToPoint1 = Math.max(extraDistanceToPoint1, -projectedDistanceToPoint);

            // // FIXME: 1/8/18 clean this up?
            interiorVertex = nextInteriorVertex;
            nextInteriorVertexIndex = interiorPolygon.getNextVertexIndex(nextInteriorVertexIndex);
            nextInteriorVertex = interiorPolygon.getVertex(nextInteriorVertexIndex);
         }


         edgeOnQ.negateDirection();
         double extraDistanceToPoint2 = 0.0;

         leftMostIndexOnInteriorPolygon = EuclidGeometryPolygonTools.findVertexIndex(interiorPolygon, true, Bound.MIN, Bound.MIN);
         interiorVertex = interiorPolygon.getVertex(leftMostIndexOnInteriorPolygon);
         nextInteriorVertexIndex = interiorPolygon.getNextVertexIndex(leftMostIndexOnInteriorPolygon);
         nextInteriorVertex = interiorPolygon.getVertex(nextInteriorVertexIndex);


         for (int j = 0; j < interiorPolygon.getNumberOfVertices(); j++)
         {
            vectorToInteriorPolygonVertex.set(interiorVertex);
            double projectedDistanceToPoint = edgeOnQ.getDirectionX() * vectorToInteriorPolygonVertex.getX() +
                  edgeOnQ.getDirectionY() * vectorToInteriorPolygonVertex.getY();

            extraDistanceToPoint2 = Math.max(extraDistanceToPoint2, -projectedDistanceToPoint);

            // // FIXME: 1/8/18 clean this up?
            interiorVertex = nextInteriorVertex;
            nextInteriorVertexIndex = interiorPolygon.getNextVertexIndex(nextInteriorVertexIndex);
            nextInteriorVertex = interiorPolygon.getVertex(nextInteriorVertexIndex);
         }


         double percentAlongSegmentVertex1 = (distanceInside + extraDistanceToPoint1) / polygonAsLineSegment.length();
         double percentAlongSegmentVertex2 = (distanceInside + extraDistanceToPoint2) / polygonAsLineSegment.length();

         // the line segment collapses to a point
         if (percentAlongSegmentVertex1 >= 0.5 && percentAlongSegmentVertex2 >= 0.5)
         {
            polygonAsLineSegment.pointBetweenEndpointsGivenPercentage(0.5, newVertex0);

            scaledPolygonToPack.clear();
            scaledPolygonToPack.addVertex(newVertex0);
            scaledPolygonToPack.update();
            return false;
         }

         // the line segment is shrunk
         polygonAsLineSegment.pointBetweenEndpointsGivenPercentage(Math.min(percentAlongSegmentVertex1, 0.5), newVertex0);
         polygonAsLineSegment.pointBetweenEndpointsGivenPercentage(1.0 - Math.min(percentAlongSegmentVertex2, 0.5), newVertex1);

         scaledPolygonToPack.clear();
         scaledPolygonToPack.addVertex(newVertex0);
         scaledPolygonToPack.addVertex(newVertex1);
         scaledPolygonToPack.update();

         return true;
      }

      if (exteriorPolygon.getNumberOfVertices() == 1)
      {
         if (distanceInside < 0.0)
         {
            scaleConvexPolygon(exteriorPolygon, distanceInside, tempPolygon);
            return scaleConvexPolygonToContainInteriorPolygon(tempPolygon, interiorPolygon, 0.0, scaledPolygonToPack);
         }
         else
         {
            scaledPolygonToPack.set(exteriorPolygon);
            return false;
         }
      }


      rays.clear();

      int leftMostIndexOnExteriorPolygon = EuclidGeometryPolygonTools.findVertexIndex(exteriorPolygon, true, Bound.MIN, Bound.MIN);
      Point2DReadOnly exteriorVertex = exteriorPolygon.getVertex(leftMostIndexOnExteriorPolygon);
      int nextExteriorVertexIndex = exteriorPolygon.getNextVertexIndex(leftMostIndexOnExteriorPolygon);
      Point2DReadOnly nextExteriorVertex = exteriorPolygon.getVertex(nextExteriorVertexIndex);

      for (int i = 0; i < exteriorPolygon.getNumberOfVertices(); i++)
      {
         edgeOnQ.set(exteriorVertex, nextExteriorVertex);
         edgeOnQ.perpendicularVector(vectorPerpendicularToEdgeOnQ);
         vectorPerpendicularToEdgeOnQ.negate();
         linePerpendicularToEdgeOnQ.set(exteriorVertex, vectorPerpendicularToEdgeOnQ);


         double extraDistance = 0.0;

         int leftMostIndexOnInteriorPolygon = EuclidGeometryPolygonTools.findVertexIndex(interiorPolygon, true, Bound.MIN, Bound.MIN);
         Point2DReadOnly interiorVertex = interiorPolygon.getVertex(leftMostIndexOnInteriorPolygon);
         int nextInteriorVertexIndex = interiorPolygon.getNextVertexIndex(leftMostIndexOnInteriorPolygon);
         Point2DReadOnly nextInteriorVertex = interiorPolygon.getVertex(nextInteriorVertexIndex);

         for (int j = 0; j < interiorPolygon.getNumberOfVertices(); j++)
         {
            vectorToInteriorPolygonVertex.set(interiorVertex);

            double distancePerpendicularToEdge = linePerpendicularToEdgeOnQ.getDirectionX() * vectorToInteriorPolygonVertex.getX() +
                  linePerpendicularToEdgeOnQ.getDirectionY() * vectorToInteriorPolygonVertex.getY();

            extraDistance = Math.max(extraDistance, -distancePerpendicularToEdge);

            // // FIXME: 1/8/18 clean this up?
            interiorVertex = nextInteriorVertex;
            nextInteriorVertexIndex = interiorPolygon.getNextVertexIndex(nextInteriorVertexIndex);
            nextInteriorVertex = interiorPolygon.getVertex(nextInteriorVertexIndex);
         }


         linePerpendicularToEdgeOnQ.pointOnLineGivenParameter(distanceInside + extraDistance, referencePoint);
         normalizedVector.set(edgeOnQ.getDirection());

         Line2D newEdge = getARay(rays.size());
         newEdge.set(referencePoint, normalizedVector);
         rays.add(newEdge);

         exteriorVertex = nextExteriorVertex;
         nextExteriorVertexIndex = exteriorPolygon.getNextVertexIndex(nextExteriorVertexIndex);
         nextExteriorVertex = exteriorPolygon.getVertex(nextExteriorVertexIndex);
      }


      boolean foundSolution = convexPolygonConstructorFromInteriorOfRays.constructFromInteriorOfRays(rays, scaledPolygonToPack);
      if (!foundSolution)
      {
         scaledPolygonToPack.clear();
         scaledPolygonToPack.addVertex(exteriorPolygon.getCentroid());
         scaledPolygonToPack.update();
      }

      return foundSolution;
   }

   /**
    * Grows or shrinks the size of the polygon, If distance is positive it shrinks the polygon in by the distance in meters,
    * If the distance is negative it grows the polygon. If polygonQ is a line and the distance is negative, a 6 point polygon is returned around the line. If
    * polygonQ is a point, a square is returned around the point. polygonQ is not changed.
    */
   public void scaleConvexPolygon(FrameConvexPolygon2DReadOnly polygonQ, double distance, FrameConvexPolygon2D framePolygonToPack)
   {
      if (Math.abs(distance) < 1.0e-10)
      {
         framePolygonToPack.setIncludingFrame(polygonQ);
         return;
      }

      framePolygonToPack.clear(polygonQ.getReferenceFrame());
      framePolygonToPack.update();
      scaleConvexPolygon((ConvexPolygon2DReadOnly) polygonQ, distance, (ConvexPolygon2DBasics) framePolygonToPack);
      framePolygonToPack.update();
   }
}
