package us.ihmc.communication;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

/**
 * TODO: Does this category make sense? Should these go in {@link HumanoidControllerAPI}?
 */
public class StateEstimatorAPI
{
   private static final ROS2Topic<HandJointAnglePacket> HAND_JOINT_ANGLES = HumanoidControllerAPI.HUMANOID_CONTROLLER.withOutput().withTypeName(HandJointAnglePacket.class);

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglesTopic(String robotName)
   {
      return HAND_JOINT_ANGLES.withRobot(robotName);
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(String robotName)
   {
      return getRobotConfigurationDataTopic(HumanoidControllerAPI.getOutputTopic(robotName));
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(ROS2Topic<?> outputTopic)
   {
      return outputTopic.withTypeName(RobotConfigurationData.class).withQoS(ROS2QosProfile.BEST_EFFORT());
   }

   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglePacketTopic(String robotName)
   {
      return getTopic(HandJointAnglePacket.class, robotName);
   }

   public static <T> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      // Input types
      if (messageClass.equals(StampedPosePacket.class)
       || messageClass.equals(PelvisPoseErrorPacket.class)
       || messageClass.equals(LocalizationPacket.class))
      {
         return HumanoidControllerAPI.getInputTopic(robotName).withTypeName(messageClass).withQoS(ROS2QosProfile.BEST_EFFORT());
      }
      // Output types
      else if (messageClass.equals(HandJointAnglePacket.class)
            || messageClass.equals(RobotConfigurationData.class))
      {
         return HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(messageClass).withQoS(ROS2QosProfile.BEST_EFFORT());
      }
      else
      {
         throw new RuntimeException("Message class is not part of the state estimator API: %s".formatted(messageClass.getName()));
      }
   }
}
