package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;

public interface FootRotationCalculator
{
   public void compute(FramePoint2d desiredCoP, FramePoint2d centerOfPressure);
   public boolean isFootRotating();
   public void getLineOfRotation(FrameLine2d lineOfRotationToPack);
   public void reset();

}
