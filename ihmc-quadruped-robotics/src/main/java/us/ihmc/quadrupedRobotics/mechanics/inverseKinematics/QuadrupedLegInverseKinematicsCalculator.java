package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedLegInverseKinematicsCalculator
{
   boolean solveForEndEffectorLocationInBodyAndUpdateDesireds(RobotQuadrant robotQuadrant, Vector3D footPositionInFrameBeforeHipRoll, FullRobotModel fullRobotModel);
}