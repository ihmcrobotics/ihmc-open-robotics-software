package us.ihmc.quadrupedRobotics.swingLegChooser;

import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface NextSwingLegChooser
{
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant swingLeg, FrameVector desiredVelocity, double desiredYawRate);
}
