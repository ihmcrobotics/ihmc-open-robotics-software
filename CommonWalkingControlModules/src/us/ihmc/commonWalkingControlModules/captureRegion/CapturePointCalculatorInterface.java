package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;

public interface CapturePointCalculatorInterface
{
   public abstract void computeCapturePoint(RobotSide supportSide);
   public abstract FramePoint computePredictedCapturePoint(RobotSide supportLeg, double captureTime, FramePoint centerOfPressure);

   public abstract FramePoint getCapturePointInFrame(ReferenceFrame referenceFrame);
   public abstract FramePoint2d getCapturePoint2dInFrame(ReferenceFrame referenceFrame);


}
