package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConvexPolygonShrinker
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

   public ConvexPolygonShrinker()
   {
      for (int i=0; i<8; i++)
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
   
   
   public boolean shrinkConstantDistanceInto(ConvexPolygon2D polygonQ, double distance, ConvexPolygon2D polygonToPack)
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

         polygonAsLineSegment.set(vertex0, vertex1);
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
   
   public void shrinkConstantDistanceInto(FrameConvexPolygon2d polygonQ, double distance, FrameConvexPolygon2d framePolygonToPack)
   {      
      if (Math.abs(distance) < 1.0e-10)
      {
         framePolygonToPack.setIncludingFrameAndUpdate(polygonQ);
         return;
      }
      
      framePolygonToPack.clear(polygonQ.getReferenceFrame());
      framePolygonToPack.update();
      ConvexPolygon2D polygon2dToPack = framePolygonToPack.getConvexPolygon2d();
      shrinkConstantDistanceInto(polygonQ.getConvexPolygon2dCopy(), distance, polygon2dToPack);
//      framePolygonToPack.updateFramePoints();
      framePolygonToPack.update();
   }
}
