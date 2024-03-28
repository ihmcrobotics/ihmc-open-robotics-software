package us.ihmc.communication;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
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
      return ROS2Tools.typeNamedTopic(RobotConfigurationData.class, ROS2Tools.getControllerOutputTopic(robotName));
   }

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglePacketTopic(String robotName)
   {
      return ROS2Tools.typeNamedTopic(HandJointAnglePacket.class, ROS2Tools.getControllerOutputTopic(robotName));
   }
}
