package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.utilities.math.geometry.Transform3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.robotSide.RobotSide;

public interface LegInverseKinematicsCalculator
{
   public abstract void solve(LegJointPositions legJointPositionsToPack, Transform3d footToPelvis, RobotSide robotSide, double desiredHipYaw) throws InverseKinematicsException;
}
