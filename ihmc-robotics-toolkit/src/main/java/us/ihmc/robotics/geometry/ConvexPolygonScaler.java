package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConvexPolygonScaler
{
   private final LineSegment2D polygonAsLineSegment = new LineSegment2D();
   private final ArrayList<Point2D> newVertices = new ArrayList<Point2D>();
   private final Point2D newVertex0 = new Point2D();
   private final Point2D newVertex1 = new Point2D();
   private final ArrayList<Line2D> rays = new ArrayList<Line2D>();

   private final Line2D edgeOnQ = new Line2D();
   private final Vector2D vectorPerpendicularToEdgeOnQ = new Vector2D();
   private final Line2D LinePerpendicularToEdgeOnQ = new Line2D();
   private final Point2D referencePoint = new Point2D();
   private final Vector2D normalizedVector = new Vector2D();
   
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
   public boolean scaleConvexPolygon(ConvexPolygon2D polygonQ, double distance, ConvexPolygon2D polygonToPack)
   {
      if (Math.abs(distance) < 1.0e-10)
      {
         polygonToPack.setAndUpdate(polygonQ);
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

         newVertices.clear();
         newVertices.add(newVertex0);
         newVertices.add(newVertex1);

         polygonToPack.setAndUpdate(newVertices, 2);
         
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
         polygonToPack.setAndUpdate(polygonQ);
         return false;
      }

      rays.clear();

      int leftMostIndexOnPolygonQ = polygonQ.getMinXIndex();
      Point2DReadOnly vertexQ = polygonQ.getVertex(leftMostIndexOnPolygonQ);
      int nextVertexQIndex = polygonQ.getNextVertexIndex(leftMostIndexOnPolygonQ);
      Point2DReadOnly nextVertexQ = polygonQ.getVertex(nextVertexQIndex);

      for (int i = 0; i < polygonQ.getNumberOfVertices(); i++)
      {
         edgeOnQ.set(vertexQ, nextVertexQ);
         edgeOnQ.perpendicularVector(vectorPerpendicularToEdgeOnQ);
         vectorPerpendicularToEdgeOnQ.negate();
         LinePerpendicularToEdgeOnQ.set(vertexQ, vectorPerpendicularToEdgeOnQ);
         LinePerpendicularToEdgeOnQ.pointOnLineGivenParameter(distance, referencePoint);
         edgeOnQ.getDirection(normalizedVector);
         
         
         Line2D newEdge = getARay(rays.size());
         newEdge.set(referencePoint, normalizedVector);
         rays.add(newEdge);

         vertexQ = nextVertexQ;
         nextVertexQIndex = polygonQ.getNextVertexIndex(nextVertexQIndex);
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
   
   /**
    * Grows or shrinks the size of the polygon, If distance is positive it shrinks the polygon in by the distance in meters,
    * If the distance is negative it grows the polygon. If polygonQ is a line and the distance is negative, a 6 point polygon is returned around the line. If
    * polygonQ is a point, a square is returned around the point. polygonQ is not changed. 
    */
   public void scaleConvexPolygon(FrameConvexPolygon2d polygonQ, double distance, FrameConvexPolygon2d framePolygonToPack)
   {      
      if (Math.abs(distance) < 1.0e-10)
      {
         framePolygonToPack.setIncludingFrameAndUpdate(polygonQ);
         return;
      }
      
      framePolygonToPack.clear(polygonQ.getReferenceFrame());
      framePolygonToPack.update();
      ConvexPolygon2D polygon2dToPack = framePolygonToPack.getConvexPolygon2d();
      scaleConvexPolygon(polygonQ.getConvexPolygon2d(), distance, polygon2dToPack);
//      framePolygonToPack.updateFramePoints();
      framePolygonToPack.update();
   }
}
