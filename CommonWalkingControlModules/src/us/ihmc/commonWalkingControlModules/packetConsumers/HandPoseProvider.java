package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface HandPoseProvider
{
   public abstract FramePose getDesiredHandPose(RobotSide robotSide);
   
   public abstract FramePose[] getDesiredHandPoses(RobotSide robotSide);

   public abstract ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide);

   public abstract double getTrajectoryTime();
   
   public abstract void clear();
   
   public abstract boolean checkForNewPose(RobotSide robotSide);

   public abstract boolean checkForNewPoseList(RobotSide robotSide);
   
   public abstract boolean checkForNewRotateAboutAxisPacket(RobotSide robotSide);
   
   public abstract Point3d getRotationAxisOriginInWorld(RobotSide robotSide);
   
   public abstract Vector3d getRotationAxisInWorld(RobotSide robotSide);
   
   public abstract HandRotateAboutAxisPacket.DataType checkHandRotateAboutAxisDataType(RobotSide robotSide);
   
   public abstract double getRotationAngleRightHandRule(RobotSide robotSide);

   public abstract boolean controlHandAngleAboutAxis(RobotSide robotSide);
   
   public abstract double getGraspOffsetFromControlFrame(RobotSide robotSide);
   
   public abstract boolean checkForHomePosition(RobotSide robotSide);

   public abstract HandPosePacket.DataType checkHandPosePacketDataType(RobotSide robotSide);

   public abstract HandPosePacket.DataType checkHandPoseListPacketDataType(RobotSide robotSide);

   public abstract Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide);

   public abstract Map<OneDoFJoint, double[]> getDesiredJointAngleForWaypointTrajectory(RobotSide robotSide);

   public abstract Vector3d getForceConstraint(RobotSide robotSide);
   
   public abstract double getTangentialForce(RobotSide robotSide);

   /**
    * By default it should be all true, meaning the hand orientation will be full constrained.
    */
   public abstract boolean[] getControlledOrientationAxes(RobotSide robotSide);

   public abstract double getPercentOfTrajectoryWithOrientationBeingControlled(RobotSide robotSide);
}