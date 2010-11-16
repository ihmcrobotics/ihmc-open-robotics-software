package us.ihmc.commonWalkingControlModules.kinematics;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;

public interface LegInverseKinematicsCalculator
{
   public abstract void solve(LegJointPositions legJointPositionsToPack, Transform3D footToPelvis, RobotSide robotSide, double desiredHipYaw) throws InverseKinematicsException;
}
