package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.robotSide.RobotSide;

public interface DesiredPelvisOrientationControlModule
{
   public abstract FrameOrientation getDesiredPelvisOrientationSingleSupportCopy(RobotSide robotSide);

   public abstract FrameOrientation getDesiredPelvisOrientationDoubleSupportCopy();

   public abstract void setDesiredPelvisOrientation(FrameOrientation orientation);
   
   public abstract FrameOrientation getEstimatedOrientationAtEndOfStepCopy(RobotSide stanceSide, FramePoint desiredFootLocation);
   
   public abstract void useTwistScale(boolean useTwistScale);
}
