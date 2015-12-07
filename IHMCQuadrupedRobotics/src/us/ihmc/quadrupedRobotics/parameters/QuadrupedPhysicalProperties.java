package us.ihmc.quadrupedRobotics.parameters;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedPhysicalProperties
{
   public Vector3d getOffsetFromKneeToFoot(RobotQuadrant robotQuadrant);
}
