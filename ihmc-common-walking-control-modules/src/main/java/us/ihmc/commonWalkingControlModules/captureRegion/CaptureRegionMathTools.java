package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;

public class CaptureRegionMathTools
{
   private final FrameVector2D rotatedFromA = new FrameVector2D();

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

      double angleBetweenDirections = directionA.angle(directionB);
      double angleBetweenDirectionsToSetLine = angleBetweenDirections * alpha;

      rotatedFromA.setReferenceFrame(directionA.getReferenceFrame());
      RotationMatrixTools.applyYawRotation(angleBetweenDirectionsToSetLine, directionA, rotatedFromA);

      rotatedFromA.scale(radius / rotatedFromA.norm());

      pointToPack.setIncludingFrame(rotatedFromA);
      pointToPack.add(centerOfCircle);
   }
}
