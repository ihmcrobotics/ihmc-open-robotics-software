package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;

public class CaptureRegionMathTools
{
   /**
   * This function computes the position of the capture point after the time dt has passed given
   * the current capture point and the Cop which is assumed to be at constant position over dt.
   */
   public static void predictCapturePoint(FramePoint2d ICP, FramePoint2d CoP, double dt, double omega0, FramePoint2d predictedICPtoPack)
   {
      // make sure everything is in the same frame:
      ICP.checkReferenceFrameMatch(CoP);
      ICP.checkReferenceFrameMatch(predictedICPtoPack);

      // CP = (ICP - CoP) * exp(omega0*dt) + CoP
      predictedICPtoPack.set(ICP);
      predictedICPtoPack.sub(CoP);
      predictedICPtoPack.scale(Math.exp(omega0 * dt));
      predictedICPtoPack.add(CoP);
   }

   /**
    * Takes a line and a circle and computes the intersection. If there is no intersection sets NaN.
    */
   public static void solveIntersectionOfRayAndCircle(FramePoint2d pointA, FramePoint2d pointB, FrameVector2d vector, double R, FramePoint2d pointToPack)
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

   private final Vector3D negZRotationAxis = new Vector3D(0.0, 0.0, -1.0);
   private final RigidBodyTransform rotation = new RigidBodyTransform();
   private final FrameVector rotatedFromA = new FrameVector();
   private final AxisAngle axisAngle = new AxisAngle();

   /**
   * Will return a point on a circle around the origin. The point will be in between the given directions and at
   * a position specified by the alpha value. E.g. an alpha value of 0.5 will result in the point being in the
   * middle of the given directions.
   */
   public void getPointBetweenVectorsAtDistanceFromOriginCircular(FrameVector2d directionA, FrameVector2d directionB, double alpha, double radius,
         FramePoint2d midpoint, FramePoint2d pointToPack)
   {
      directionA.checkReferenceFrameMatch(directionB.getReferenceFrame());
      directionA.checkReferenceFrameMatch(midpoint.getReferenceFrame());
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);

      double angleBetweenDirections = Math.abs(directionA.angle(directionB));
      double angleBetweenDirectionsToSetLine = angleBetweenDirections * alpha;

      rotatedFromA.setToZero(directionA.getReferenceFrame());
      rotatedFromA.set(directionA.getX(), directionA.getY(), 0.0);

      axisAngle.set(negZRotationAxis, angleBetweenDirectionsToSetLine);
      rotation.setRotation(axisAngle);
      rotatedFromA.applyTransform(rotation);

      rotatedFromA.normalize();
      rotatedFromA.scale(radius);

      pointToPack.changeFrame(rotatedFromA.getReferenceFrame());
      pointToPack.set(rotatedFromA.getX(), rotatedFromA.getY());
      pointToPack.add(midpoint);
   }
}
