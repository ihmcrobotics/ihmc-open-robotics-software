package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.robotics.geometry.FrameLine2d;

public interface FootRotationCalculator
{
   public void compute(FramePoint2D desiredCoP, FramePoint2D centerOfPressure);
   public boolean isFootRotating();
   public void getLineOfRotation(FrameLine2d lineOfRotationToPack);
   public void reset();

}
