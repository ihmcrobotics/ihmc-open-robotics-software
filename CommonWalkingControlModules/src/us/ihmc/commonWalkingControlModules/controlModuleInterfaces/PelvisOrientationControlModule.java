package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface PelvisOrientationControlModule
{
   public abstract FrameVector computePelvisTorque(RobotSide supportLeg, FrameOrientation desiredPelvisOrientation);
}
