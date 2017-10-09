package us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface NextSwingLegChooser
{
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant swingLeg, FrameVector3D desiredVelocity, double desiredYawRate);
}
