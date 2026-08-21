package us.ihmc.communication;

import controller_msgs.HandConfigurationCommandMessage;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Robot-agnostic hand configuration commands. The walking / RL controller publishes an integer
 * configuration; the robot's hand controller interprets it.
 */
public final class HandConfigurationAPI
{
   private static final ROS2Topic<?> ROOT_TOPIC = new ROS2Topic<>("/ihmc/hand_configuration");

   private HandConfigurationAPI()
   {
   }

   public static ROS2Topic<HandConfigurationCommandMessage> getCommandTopic(RobotSide robotSide)
   {
      return ROOT_TOPIC.appendedWith(robotSide.getLowerCaseName())
                       .appendedWith("hand_configuration_command")
                       .withType(HandConfigurationCommandMessage.class)
                       .withQoS(ROS2QoSProfile.RELIABLE);
   }
}
