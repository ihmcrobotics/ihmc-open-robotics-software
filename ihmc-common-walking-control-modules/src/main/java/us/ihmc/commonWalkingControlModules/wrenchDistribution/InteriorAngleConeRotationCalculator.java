package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class InteriorAngleConeRotationCalculator implements FrictionConeRotationCalculator
{
   private final double offset;

   public InteriorAngleConeRotationCalculator(double offset)
   {
      this.offset = offset;
   }

   @Override
   public double computeConeRotation(YoPlaneContactState yoPlaneContactState, int contactPointIndex)
   {
      yoPlaneContactState.getContactPoints().get(contactPointIndex).getPosition2d(point2d);
      ConvexPolygon2D supportPolygon = yoPlaneContactState.getSupportPolygonInPlaneFrame();
      int vertexIndex = supportPolygon.getClosestVertexIndex(point2d);

      double angleOfEdge = getAngleOfEdgeAfterPoint(supportPolygon, vertexIndex);
      double interiorAngle = getInteriorAngle(supportPolygon, vertexIndex);
      return angleOfEdge - 0.5 * interiorAngle + offset;
   }


   private final Point2D point2d = new Point2D();
   private final LineSegment2D edge1 = new LineSegment2D();
   private final LineSegment2D edge2 = new LineSegment2D();
   private final Vector2D edgeDirection1 = new Vector2D();
   private final Vector2D edgeDirection2 = new Vector2D();

   public double getInteriorAngle(ConvexPolygon2D supportPolygon, int vertexIndex)
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

   public double getAngleOfEdgeAfterPoint(ConvexPolygon2D supportPolygon, int vertexIndex)
   {
      supportPolygon.getEdge(vertexIndex, edge);
      edgeDirection.sub(edge.getSecondEndpoint(), edge.getFirstEndpoint());
      xAxis.set(1.0, 0.0);
      return xAxis.angle(edgeDirection);
   }

}
