package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface DesiredPelvisOrientationControlModule
{
   public abstract FrameOrientation getDesiredPelvisOrientationSingleSupportCopy(RobotSide robotSide);

   public abstract FrameOrientation getDesiredPelvisOrientationDoubleSupportCopy();

   public abstract void setDesiredPelvisOrientation(FrameOrientation orientation);
   
   public abstract FrameOrientation getEstimatedOrientationAtEndOfStepCopy(RobotSide stanceSide, FramePoint desiredFootLocation);
   
   public abstract void useTwistScale(boolean useTwistScale);
}
