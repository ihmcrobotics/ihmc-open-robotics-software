package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedLegInverseKinematicsCalculator
{
   boolean solveForEndEffectorLocationInBodyAndUpdateDesireds(RobotQuadrant robotQuadrant, Vector3d footPositionInFrameBeforeHipRoll, FullRobotModel fullRobotModel);
}