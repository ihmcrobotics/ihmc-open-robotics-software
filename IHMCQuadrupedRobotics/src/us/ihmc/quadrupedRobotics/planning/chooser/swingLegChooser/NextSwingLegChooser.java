package us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser;

import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface NextSwingLegChooser
{
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant swingLeg, FrameVector desiredVelocity, double desiredYawRate);
}
