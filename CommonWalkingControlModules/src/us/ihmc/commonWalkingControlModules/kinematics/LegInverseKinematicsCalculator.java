package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;

public interface LegInverseKinematicsCalculator
{
   public abstract void solve(LegJointPositions legJointPositionsToPack, RigidBodyTransform footToPelvis, RobotSide robotSide, double desiredHipYaw) throws InverseKinematicsException;
}
