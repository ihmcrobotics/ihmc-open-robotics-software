package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;

public interface CapturePointCalculator
{
   void compute(FramePoint2DBasics capturePointToPack, double omega0);
}
