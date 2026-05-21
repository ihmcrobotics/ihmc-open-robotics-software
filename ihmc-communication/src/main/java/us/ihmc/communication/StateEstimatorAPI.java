package us.ihmc.communication;

import controller_msgs.HandJointAnglePacket;
import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import controller_msgs.RobotConfigurationData;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;

/**
 * TODO: Does this category make sense? Should these go in {@link HumanoidControllerAPI}?
 */
public final class StateEstimatorAPI
{
   public static ROS2Topic<HandJointAnglePacket> getHandJointAnglesTopic(String robotName)
   {
      return getTopic(HandJointAnglePacket.class, robotName);
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(String robotName)
   {
      return getRobotConfigurationDataTopic(HumanoidControllerAPI.getOutputTopic(robotName));
   }

   public static ROS2Topic<RobotConfigurationData> getRobotConfigurationDataTopic(ROS2Topic<?> outputTopic)
   {
      return outputTopic.withType(RobotConfigurationData.class);
   }

   public static <T extends ROS2Message<T>> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      // Input types
      if (messageClass.equals(StampedPosePacket.class)
       || messageClass.equals(PelvisPoseErrorPacket.class)
       || messageClass.equals(LocalizationPacket.class))
      {
         return HumanoidControllerAPI.getInputTopic(robotName).withType(messageClass);
      }
      // Output types
      else if (messageClass.equals(HandJointAnglePacket.class)
            || messageClass.equals(RobotConfigurationData.class))
      {
         return HumanoidControllerAPI.getOutputTopic(robotName).withType(messageClass);
      }
      else
      {
         throw new RuntimeException("Message class is not part of the state estimator API: %s".formatted(messageClass.getName()));
      }
   }
}
