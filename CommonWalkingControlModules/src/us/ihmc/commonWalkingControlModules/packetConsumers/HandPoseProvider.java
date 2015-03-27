package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface HandPoseProvider
{
   public abstract FramePose getDesiredHandPose(RobotSide robotSide);
   
   public abstract FramePose[] getDesiredHandPoses(RobotSide robotSide);

   public abstract ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide);

   public abstract double getTrajectoryTime();
   
   public abstract boolean checkForNewPose(RobotSide robotSide);

   public abstract boolean checkForNewPoseList(RobotSide robotSide);
   
   public abstract boolean checkForNewRotateAboutAxisPacket(RobotSide robotSide);
   
   public abstract Point3d getRotationAxisOriginInWorld(RobotSide robotSide);
   
   public abstract Vector3d getRotationAxisInWorld(RobotSide robotSide);
   
   public abstract double getRotationAngleRightHandRule(RobotSide robotSide);

   public abstract boolean checkForNewPauseCommand(RobotSide robotSide);

   public abstract boolean checkForHomePosition(RobotSide robotSide);

   public abstract HandPosePacket.DataType checkHandPosePacketDataType(RobotSide robotSide);

   public abstract HandPosePacket.DataType checkHandPoseListPacketDataType(RobotSide robotSide);

   public abstract Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide);

   public abstract Map<OneDoFJoint, double[]> getDesiredJointAngleForWaypointTrajectory(RobotSide robotSide);

   public abstract void getPauseCommand(RobotSide robotSide);

   public abstract boolean checkForNewWholeBodyPoseList(RobotSide robotSide);

   public abstract double[] getDesiredWholeBodyTrajectoryTimeArray();
   
   public abstract double[][] getDesiredWholeBodyTrajectoryPositionArray(RobotSide robotSide);
   
   public abstract double[][] getDesiredWholeBodyTrajectoryVelocityArray(RobotSide robotSide);
   
   public abstract void setWholeBodyTrajectoryPacketAtomicReferenceToNull(RobotSide robotSide);

   public abstract boolean checkForNewArmJointTrajectory(RobotSide robotSide);

   public abstract ArmJointTrajectoryPacket getArmJointTrajectoryPacket(RobotSide robotSide);
}