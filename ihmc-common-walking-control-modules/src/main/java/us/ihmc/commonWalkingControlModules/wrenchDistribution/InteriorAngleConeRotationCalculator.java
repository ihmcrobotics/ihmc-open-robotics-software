package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class InteriorAngleConeRotationCalculator implements FrictionConeRotationCalculator
{
   private final double offset;

   public InteriorAngleConeRotationCalculator(double offset)
   {
      this.offset = offset;
   }

   @Override
   public double computeConeRotation(ConvexPolygon2DReadOnly supportPolygonInPlaneFrame, Point3DReadOnly contactPoint)
   {
      point2d.set(contactPoint);
      int vertexIndex = supportPolygonInPlaneFrame.getClosestVertexIndex(point2d);

      double angleOfEdge = getAngleOfEdgeAfterPoint(supportPolygonInPlaneFrame, vertexIndex);
      double interiorAngle = getInteriorAngle(supportPolygonInPlaneFrame, vertexIndex);
      return angleOfEdge - 0.5 * interiorAngle + offset;
   }

   private final Point2D point2d = new Point2D();
   private final LineSegment2D edge1 = new LineSegment2D();
   private final LineSegment2D edge2 = new LineSegment2D();
   private final Vector2D edgeDirection1 = new Vector2D();
   private final Vector2D edgeDirection2 = new Vector2D();

   public double getInteriorAngle(ConvexPolygon2DReadOnly supportPolygon, int vertexIndex)
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

   public double getAngleOfEdgeAfterPoint(ConvexPolygon2DReadOnly supportPolygon, int vertexIndex)
   {
      supportPolygon.getEdge(vertexIndex, edge);
      edgeDirection.sub(edge.getSecondEndpoint(), edge.getFirstEndpoint());
      xAxis.set(1.0, 0.0);
      return xAxis.angle(edgeDirection);
   }

}
