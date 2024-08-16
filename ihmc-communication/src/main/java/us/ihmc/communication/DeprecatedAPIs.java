package us.ihmc.communication;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.ros2.ROS2Topic;

@Deprecated
public final class DeprecatedAPIs
{
   public static final String BEHAVIOR_MODULE_NAME = "behavior";

   public static final ROS2Topic<?> BEHAVIOR_MODULE = ROS2Tools.IHMC_ROOT.withModule(BEHAVIOR_MODULE_NAME);

   private static final ROS2Topic<HandDesiredConfigurationMessage> HAND_CONFIGURATION
         = HumanoidControllerAPI.HUMANOID_CONTROLLER.withInput().withTypeName(HandDesiredConfigurationMessage.class);

   public static ROS2Topic<HandDesiredConfigurationMessage> getHandConfigurationTopic(String robotName)
   {
      return HAND_CONFIGURATION.withRobot(robotName);
   }
}
