package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredPelvisOrientationControlModule
{
   public abstract FrameOrientation getDesiredPelvisOrientationSingleSupportCopy(RobotSide robotSide);

   public abstract FrameOrientation getDesiredPelvisOrientationDoubleSupportCopy();

   public abstract void setDesiredPelvisOrientation(FrameOrientation orientation);
   
   public abstract FrameOrientation getEstimatedOrientationAtEndOfStepCopy(RobotSide stanceSide, FramePoint desiredFootLocation);
   
   public abstract void useTwistScale(boolean useTwistScale);
}
