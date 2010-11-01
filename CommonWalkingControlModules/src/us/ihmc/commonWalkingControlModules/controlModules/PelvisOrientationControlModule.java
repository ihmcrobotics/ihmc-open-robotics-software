package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;

public interface PelvisOrientationControlModule
{
   public abstract FrameVector computePelvisTorque(RobotSide supportLeg, Orientation desiredPelvisOrientation);
}
