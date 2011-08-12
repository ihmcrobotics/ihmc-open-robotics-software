package us.ihmc.commonWalkingControlModules.kinematics;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointPositions;
import us.ihmc.robotSide.RobotSide;

public interface ArmInverseKinematicsCalculator
{
   public abstract void solve(ArmJointPositions armJointPositionsToPack, Transform3D handToChest, RobotSide robotSide, Vector3d elbowPoint) throws InverseKinematicsException;
}