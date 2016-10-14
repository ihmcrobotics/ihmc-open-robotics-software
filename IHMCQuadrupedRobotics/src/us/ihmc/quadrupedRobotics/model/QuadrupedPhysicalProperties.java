package us.ihmc.quadrupedRobotics.model;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public interface QuadrupedPhysicalProperties
{
   Vector3d getOffsetFromJointBeforeFootToSole(RobotQuadrant robotQuadrant);
   ArrayList<Point2d> getFootGroundContactPoints(RobotQuadrant robotQuadrant);
}
