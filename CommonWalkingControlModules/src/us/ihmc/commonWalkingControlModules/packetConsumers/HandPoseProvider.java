package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.Map;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface HandPoseProvider
{
   public abstract FramePose getDesiredHandPose(RobotSide robotSide);

   public abstract ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide);

   public abstract double getTrajectoryTime();

   public abstract boolean checkForNewPose(RobotSide robotSide);

   public abstract boolean checkForHomePosition(RobotSide robotSide);

   public abstract HandPosePacket.DataType checkPacketDataType(RobotSide robotSide);

   public abstract Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide);
}