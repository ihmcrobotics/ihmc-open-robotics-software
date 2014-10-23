package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.robotSide.RobotSide;

public interface PelvisOrientationControlModule
{
   public abstract FrameVector computePelvisTorque(RobotSide supportLeg, FrameOrientation desiredPelvisOrientation);
}
