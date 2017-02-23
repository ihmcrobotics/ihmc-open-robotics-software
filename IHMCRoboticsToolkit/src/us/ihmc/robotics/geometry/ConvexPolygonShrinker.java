package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConvexPolygonShrinker
{
   private final LineSegment2d polygonAsLineSegment = new LineSegment2d();
   private final ArrayList<Point2D> newVertices = new ArrayList<Point2D>();
   private final Point2D newVertex0 = new Point2D();
   private final Point2D newVertex1 = new Point2D();
   private final ArrayList<Line2d> rays = new ArrayList<Line2d>();

   private final Line2d edgeOnQ = new Line2d();
   private final Vector2D vectorPerpendicularToEdgeOnQ = new Vector2D();
   private final Line2d LinePerpendicularToEdgeOnQ = new Line2d();
   private final Point2D referencePoint = new Point2D();
   private final Vector2D normalizedVector = new Vector2D();
   
   private final ArrayList<Line2d> edgePool = new ArrayList<Line2d>();
   
   private final ConvexPolygonConstructorFromInteriorOfRays convexPolygonConstructorFromInteriorOfRays = new ConvexPolygonConstructorFromInteriorOfRays();

   public ConvexPolygonShrinker()
   {
      for (int i=0; i<8; i++)
      {
         edgePool.add(new Line2d());
      }
   }
   
   private Line2d getARay(int index)
   {
      if (edgePool.size() <= index)
      {
         for (int i=0; i<index - edgePool.size() + 1; i++)
         {
            edgePool.add(new Line2d());
         }
      }
      
      return edgePool.get(index);
   }
   
   
   public boolean shrinkConstantDistanceInto(ConvexPolygon2d polygonQ, double distance, ConvexPolygon2d polygonToPack)
   {
      if (Math.abs(distance) < 1.0e-10)
      {
         polygonToPack.setAndUpdate(polygonQ);
         return true;
      }

      if (polygonQ.hasExactlyTwoVertices())
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

         polygonAsLineSegment.pointBetweenEndPointsGivenParameter(percentageAlongSegment, newVertex0);
         polygonAsLineSegment.pointBetweenEndPointsGivenParameter(1 - percentageAlongSegment, newVertex1);

         newVertices.clear();
         newVertices.add(newVertex0);
         newVertices.add(newVertex1);

         polygonToPack.setAndUpdate(newVertices, 2);
         
         return true;
      }

      if (polygonQ.hasExactlyOneVertex())
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
         LinePerpendicularToEdgeOnQ.set(vertexQ, vectorPerpendicularToEdgeOnQ);
         LinePerpendicularToEdgeOnQ.getPointGivenParameter(distance, referencePoint);
         edgeOnQ.getNormalizedVector(normalizedVector);
         
         
         Line2d newEdge = getARay(rays.size());
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
      ConvexPolygon2d polygon2dToPack = framePolygonToPack.getConvexPolygon2d();
      shrinkConstantDistanceInto(polygonQ.getConvexPolygon2dCopy(), distance, polygon2dToPack);
//      framePolygonToPack.updateFramePoints();
      framePolygonToPack.update();
   }
}
