package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;

public interface FootRotationCalculator
{
   public void compute(FramePoint2D desiredCoP, FramePoint2D centerOfPressure);
   public boolean isFootRotating();
   public void getLineOfRotation(FrameLine2D lineOfRotationToPack);
   public void reset();

}
