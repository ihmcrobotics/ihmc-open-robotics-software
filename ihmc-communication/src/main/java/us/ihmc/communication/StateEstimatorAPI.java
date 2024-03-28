package us.ihmc.communication;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public class StateEstimatorAPI
{
   private static final ROS2Topic<HandJointAnglePacket> HAND_JOINT_ANGLES = ROS2Tools.HUMANOID_CONTROLLER.withOutput().withTypeName(HandJointAnglePacket.class);

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglesTopic(String robotName)
   {
      return HAND_JOINT_ANGLES.withRobot(robotName);
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(String robotName)
   {
      return ROS2Tools.getControllerOutputTopic(robotName).withTypeName(RobotConfigurationData.class).withQoS(ROS2QosProfile.BEST_EFFORT());
   }

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglePacketTopic(String robotName)
   {
      return ROS2Tools.getControllerOutputTopic(robotName).withTypeName(HandJointAnglePacket.class).withQoS(ROS2QosProfile.BEST_EFFORT());
   }
}
