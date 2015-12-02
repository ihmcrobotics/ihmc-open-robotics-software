package us.ihmc.quadrupedRobotics.inverseKinematics;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedLegInverseKinematicsCalculator
{

   boolean solveForEndEffectorLocationInBodyAndUpdateDesireds(RobotQuadrant robotQuadrant, Vector3d footPositionInFrameBeforeHipRoll, SDFFullRobotModel fullRobotModel);

}