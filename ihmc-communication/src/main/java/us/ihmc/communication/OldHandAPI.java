package us.ihmc.communication;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.ros2.ROS2Topic;

/**
 * This is how the Atlas Robotiq and Valkyrie hands are controlled.
 * The Sake gripper and newer developments use different messaging.
 * @see SakeHandAPI
 */
public class OldHandAPI
{
   public static ROS2Topic<HandDesiredConfigurationMessage> getHandDesiredConfigurationTopic(String robotName)
   {
      return HumanoidControllerAPI.getInputTopic(robotName).withTypeName(HandDesiredConfigurationMessage.class);
   }
}
