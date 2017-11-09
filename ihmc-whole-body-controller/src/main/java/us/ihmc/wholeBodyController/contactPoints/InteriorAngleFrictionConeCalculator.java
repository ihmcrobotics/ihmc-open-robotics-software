package us.ihmc.wholeBodyController.contactPoints;

import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class InteriorAngleFrictionConeCalculator implements FrictionConeRotationCalculator
{
   private final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();

   @Override
   public double computeYawOffset(List<Point2D> contactPoints, int contactIdx, int vectors, int vectorIdx)
   {
      supportPolygon.setAndUpdate(contactPoints, contactPoints.size());
      Point2D contactPoint = contactPoints.get(contactIdx);
      int vertexIndex = supportPolygon.getClosestVertexIndex(contactPoint);

      double angleOfEdge = getAngleOfEdgeAfterPoint(vertexIndex);
      double interiorAngle = getInteriorAngle(vertexIndex);
      return angleOfEdge - 0.5 * interiorAngle;
   }

   private final LineSegment2D edge1 = new LineSegment2D();
   private final LineSegment2D edge2 = new LineSegment2D();
   private final Vector2D edgeDirection1 = new Vector2D();
   private final Vector2D edgeDirection2 = new Vector2D();

   public double getInteriorAngle(int vertexIndex)
   {
      supportPolygon.getEdge(supportPolygon.getPreviousVertexIndex(vertexIndex), edge1);
      supportPolygon.getEdge(vertexIndex, edge2);
      edgeDirection1.sub(edge1.getSecondEndpoint(), edge1.getFirstEndpoint());
      edgeDirection2.sub(edge2.getSecondEndpoint(), edge2.getFirstEndpoint());
      double angle = edgeDirection2.angle(edgeDirection1);
      double dotProduct = edgeDirection2.dot(edgeDirection1);
      if (dotProduct > 0.0)
      {
         angle = Math.PI - angle;
      }
      return angle;
   }

   private final LineSegment2D edge = new LineSegment2D();
   private final Vector2D edgeDirection = new Vector2D();
   private final Vector2D xAxis = new Vector2D();

   public double getAngleOfEdgeAfterPoint(int vertexIndex)
   {
      supportPolygon.getEdge(vertexIndex, edge);
      edgeDirection.sub(edge.getSecondEndpoint(), edge.getFirstEndpoint());
      xAxis.set(1.0, 0.0);
      return xAxis.angle(edgeDirection);
   }
}
