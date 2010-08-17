package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface CapturePointCalculatorInterface
{

   public abstract FramePoint2d getCapturePoint2dInFrame(ReferenceFrame supportAnkleZUpFrame);

   public void computePredictedCapturePoint(RobotSide supportLeg, double captureTime, FramePoint CoPInBodyZUp, FrameLineSegment2d guideLine);

   public abstract void hidePredictedCapturePoint();

   public abstract FramePoint getPredictedCapturePointInFrame(ReferenceFrame referenceFrame);
   

}
