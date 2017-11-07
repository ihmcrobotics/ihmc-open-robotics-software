package us.ihmc.wholeBodyController.contactPoints;

import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class CenteredFrictionConeCalculator implements FrictionConeRotationCalculator
{
   private final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
   private final Vector2D centroidToPoint = new Vector2D();
   private final Vector2D xAxis = new Vector2D(1.0, 0.0);

   @Override
   public double computeYawOffset(List<Point2D> contactPoints, int contactIdx, int vectors, int vectorIdx)
   {
      supportPolygon.setAndUpdate(contactPoints, contactPoints.size());
      Point2DReadOnly centroid = supportPolygon.getCentroid();
      Point2D contactPoint = contactPoints.get(contactIdx);
      centroidToPoint.sub(contactPoint, centroid);
      double angle = centroidToPoint.angle(xAxis);
      double dotProduct = centroidToPoint.dot(xAxis);
      if (dotProduct < 0.0)
      {
         angle = Math.PI - angle;
      }
      return angle;
   }

}
