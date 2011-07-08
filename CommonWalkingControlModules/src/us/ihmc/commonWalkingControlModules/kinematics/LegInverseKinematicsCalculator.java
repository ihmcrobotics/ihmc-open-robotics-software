package us.ihmc.commonWalkingControlModules.kinematics;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.robotSide.RobotSide;

public interface LegInverseKinematicsCalculator
{
   public abstract void solve(LegJointPositions legJointPositionsToPack, Transform3D footToPelvis, RobotSide robotSide, double desiredHipYaw) throws InverseKinematicsException;
}
