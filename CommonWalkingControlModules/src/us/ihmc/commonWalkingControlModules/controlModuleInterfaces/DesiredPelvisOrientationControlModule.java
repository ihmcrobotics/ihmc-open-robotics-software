package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface DesiredPelvisOrientationControlModule
{
   public abstract FrameOrientation getDesiredPelvisOrientationSingleSupport(RobotSide robotSide);

   public abstract FrameOrientation getDesiredPelvisOrientationDoubleSupport();

   public abstract void setDesiredPelvisOrientation(FrameOrientation orientation);
   
   public abstract FrameOrientation getEstimatedOrientationAtEndOfStep(RobotSide stanceSide, FramePoint desiredFootLocation);
   
   public abstract void useTwistScale(boolean useTwistScale);
}
