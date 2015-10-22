package us.ihmc.quadrupedRobotics.footstepChooser;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface SwingTargetGenerator
{
   public abstract void getSwingTarget(RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, FramePoint swingTargetToPack, double desiredYawRate);
}
