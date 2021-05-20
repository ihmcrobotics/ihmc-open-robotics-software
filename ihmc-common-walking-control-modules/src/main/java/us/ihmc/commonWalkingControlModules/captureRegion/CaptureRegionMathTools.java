package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;

public class CaptureRegionMathTools
{
   /**
    * Takes a line and a circle and computes the intersection. If there is no intersection sets NaN.
    */
   public static void solveIntersectionOfRayAndCircle(FramePoint2D pointA, FramePoint2D pointB, FrameVector2D vector, double R, FramePoint2D pointToPack)
   {
      // Look at JPratt Notes February 18, 2009 for details on the following:

      pointA.checkReferenceFrameMatch(pointB);
      pointA.checkReferenceFrameMatch(vector);

      double Ax = pointA.getX();
      double Ay = pointA.getY();

      double Bx = pointB.getX();
      double By = pointB.getY();

      double vx = vector.getX();
      double vy = vector.getY();

      double A = (vx * vx + vy * vy);
      double B = (2.0 * vx * (Bx - Ax) + 2.0 * vy * (By - Ay));
      double C = (Bx - Ax) * (Bx - Ax) + (By - Ay) * (By - Ay) - R * R;

      double insideSqrt = B * B - 4 * A * C;

      if (insideSqrt < 0.0)
      {
         pointToPack.set(Double.NaN, Double.NaN);
         return;
      }

      double l2 = (-B + Math.sqrt(insideSqrt)) / (2.0 * A);

      pointToPack.changeFrame(pointA.getReferenceFrame());
      pointToPack.set(pointB.getX() + l2 * vector.getX(), pointB.getY() + l2 * vector.getY());
   }

   private final RigidBodyTransform rotation = new RigidBodyTransform();
   private final FrameVector3D rotatedFromA = new FrameVector3D();

   /**
   * Will return a point on a circle around the origin. The point will be in between the given directions and at
   * a position specified by the alpha value. E.g. an alpha value of 0.5 will result in the point being in the
   * middle of the given directions.
   */
   public void getPointBetweenVectorsAtDistanceFromOriginCircular(FrameVector2DReadOnly directionA,
                                                                  FrameVector2DReadOnly directionB,
                                                                  double alpha,
                                                                  double radius,
                                                                  FramePoint2DReadOnly centerOfCircle,
                                                                  FramePoint2DBasics pointToPack)
   {
      directionA.checkReferenceFrameMatch(directionB.getReferenceFrame());
      directionA.checkReferenceFrameMatch(centerOfCircle.getReferenceFrame());
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      double angleBetweenDirections = Math.abs(directionA.angle(directionB));
      double angleBetweenDirectionsToSetLine = angleBetweenDirections * alpha;

      rotatedFromA.setIncludingFrame(directionA, 0.0);

      rotation.setRotationYawAndZeroTranslation(angleBetweenDirectionsToSetLine);
      rotatedFromA.applyTransform(rotation);

      rotatedFromA.scale(radius / rotatedFromA.length());

      pointToPack.setIncludingFrame(rotatedFromA);
      pointToPack.add(centerOfCircle);
   }
}
