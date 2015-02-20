package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.ArrayList;
import java.util.Map;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface HandPoseProvider
{
   public abstract FramePose getDesiredHandPose(RobotSide robotSide);

   public abstract ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide);

   public abstract double getTrajectoryTime();

   public abstract boolean checkForNewPose(RobotSide robotSide);

   public abstract boolean checkForNewPoseList(RobotSide robotSide);

   public abstract boolean checkForNewPauseCommand(RobotSide robotSide);

   public abstract boolean checkForHomePosition(RobotSide robotSide);

   public abstract HandPosePacket.DataType checkPacketDataType(RobotSide robotSide);

   public abstract Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide);

   public abstract Map<OneDoFJoint, double[]> getDesiredJointAngleForWaypointTrajectory(RobotSide robotSide);

   public abstract void getPauseCommand(RobotSide robotSide);

   public abstract boolean checkForNewWholeBodyPoseList(RobotSide robotSide);

//   public abstract void getDesiredsForWholeBodyWaypointTrajectory(RobotSide robotSide, double[] timeArrayToPack, double[][] positionArrayToPack, double[][] velocityArrayToPack);
   
   public abstract double[] getDesiredWholeBodyTrajectoryTimeArray();
   
   public abstract double[][] getDesiredWholeBodyTrajectoryPositionArray(RobotSide robotSide);
   
   public abstract double[][] getDesiredWholeBodyTrajectoryVelocityArray(RobotSide robotSide);
   
   public abstract void setWholeBodyTrajectoryPacketAtomicReferenceToNull(RobotSide robotSide);
   
}