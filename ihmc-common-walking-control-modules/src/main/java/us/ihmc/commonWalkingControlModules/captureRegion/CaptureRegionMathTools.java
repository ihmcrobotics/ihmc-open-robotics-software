package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class CaptureRegionMathTools
{
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
      double angleBetweenDirectionsToSetLine = -angleBetweenDirections * alpha;

      rotatedFromA.setIncludingFrame(directionA, 0.0);

      rotation.setRotationYawAndZeroTranslation(angleBetweenDirectionsToSetLine);
      rotatedFromA.applyTransform(rotation);

      rotatedFromA.scale(radius / rotatedFromA.length());

      pointToPack.setIncludingFrame(rotatedFromA);
      pointToPack.add(centerOfCircle);
   }
}
