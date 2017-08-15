package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2D;

public interface FootRotationCalculator
{
   public void compute(FramePoint2D desiredCoP, FramePoint2D centerOfPressure);
   public boolean isFootRotating();
   public void getLineOfRotation(FrameLine2d lineOfRotationToPack);
   public void reset();

}
