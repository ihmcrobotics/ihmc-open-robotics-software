package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.Orientation;

public interface DesiredPelvisOrientationControlModule
{
   public abstract Orientation getDesiredPelvisOrientationSingleSupport(RobotSide robotSide);

   public abstract Orientation getDesiredPelvisOrientationDoubleSupport();

   public abstract void setDesiredPelvisOrientation(Orientation orientation);
   
   public abstract Orientation getEstimatedOrientationAtEndOfStep(RobotSide stanceSide, FramePoint desiredFootLocation);
}
