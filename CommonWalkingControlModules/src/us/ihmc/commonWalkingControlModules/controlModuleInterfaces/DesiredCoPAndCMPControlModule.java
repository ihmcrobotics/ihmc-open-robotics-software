package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Momentum;

public interface DesiredCoPAndCMPControlModule
{
   public abstract void compute(FramePoint2d capturePoint, RobotSide supportLeg, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity, FrameOrientation desiredPelvisOrientation, double omega0, Momentum momentum);
   public abstract void packCoP(FramePoint2d cop);
   public abstract void packCMP(FramePoint2d cmp);
}
