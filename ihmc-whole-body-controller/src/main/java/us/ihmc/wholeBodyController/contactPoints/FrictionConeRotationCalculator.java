package us.ihmc.wholeBodyController.contactPoints;

import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public interface FrictionConeRotationCalculator
{
   default public void packVector(List<Point2D> contactPoints, int contactIdx, int vectors, int vectorIdx, double friction,
                                  Vector3D directionToPack)
   {
      double angleOffset = computeYawOffset(contactPoints, contactIdx, vectors, vectorIdx);
      double angle = angleOffset  + 2.0 * Math.PI * vectorIdx / vectors;
      double x = Math.cos(angle) * friction;
      double y = Math.sin(angle) * friction;
      directionToPack.set(x, y, 1.0);
      directionToPack.normalize();
   }

   public abstract double computeYawOffset(List<Point2D> contactPoints, int contactIdx, int vectors, int vectorIdx);
}
