package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public interface CapturePointCalculatorInterface
{
   public abstract void computeCapturePoint(RobotSide supportSide);
   public abstract FramePoint computePredictedCapturePoint(RobotSide supportLeg, double captureTime, FramePoint centerOfPressure);

   public abstract FramePoint getCapturePointInFrame(ReferenceFrame referenceFrame);
   public abstract FramePoint2d getCapturePoint2dInFrame(ReferenceFrame referenceFrame);


}
