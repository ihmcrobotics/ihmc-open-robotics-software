package us.ihmc.commonWalkingControlModules.captureRegion;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CaptureRegionMath
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
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
    * (Function taken from the CaptureRegionCalculator class.)
    * Takes a line and a circle and computes the intersection. If there is no intersection returns null.
    */
   public static FramePoint2d solveIntersectionOfRayAndCircle(FramePoint2d pointA,
                                                              FramePoint2d pointB,
                                                              FrameVector2d vector,
                                                              double R)
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
      return null;
      
      double l2 = (-B + Math.sqrt(insideSqrt)) / (2.0 * A);
      
      FramePoint2d ret = new FramePoint2d();
      ret.set(pointA.getReferenceFrame(), pointB.getX() + l2 * vector.getX(), pointB.getY() + l2 * vector.getY());
      
      return ret;
   }
   
   private static final Vector3d negZRotationAxis = new Vector3d(0.0, 0.0, -1.0);
   private static final Transform3D rotation = new Transform3D();
   private static final FrameVector rotatedFromA = new FrameVector(worldFrame);
   private static final AxisAngle4d axisAngle = new AxisAngle4d();
   /**
   * (Function taken from the CaptureRegionCalculator class.)
   * Will return a point on a circle around the origin. The point will be in between the given directions and at
   * a position specified by the alpha value. E.g. an alpha value of 0.5 will result in the point being in the
   * middle of the given directions.
   */
   public static FramePoint2d getPointBetweenVectorsAtDistanceFromOriginCircular(FrameVector2d directionA,
                                                                                 FrameVector2d directionB,
                                                                                 double alpha,
                                                                                 double radius,
                                                                                 FramePoint2d midpoint)
   {
      directionA.checkReferenceFrameMatch(directionB.getReferenceFrame());
      directionA.checkReferenceFrameMatch(midpoint.getReferenceFrame());
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
     
      double angleBetweenDirections = directionA.angle(directionB);
      double angleBetweenDirectionsToSetLine = angleBetweenDirections * alpha;
     
      rotatedFromA.changeFrame(directionA.getReferenceFrame());
      rotatedFromA.set(directionA.getX(), directionA.getY(), 0.0);
      
      axisAngle.set(negZRotationAxis, angleBetweenDirectionsToSetLine);
      rotation.setRotation(axisAngle);
      rotatedFromA.applyTransform(rotation);
     
      rotatedFromA.normalize();
      rotatedFromA.scale(radius);
     
      FramePoint2d ret = new FramePoint2d();
      ret.changeFrame(rotatedFromA.getReferenceFrame());
      ret.set(rotatedFromA.getX(), rotatedFromA.getY());
      ret.add(midpoint);
     
      return ret;
   }
}
