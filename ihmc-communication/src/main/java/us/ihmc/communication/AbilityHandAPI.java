package us.ihmc.communication;

import controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;

public final class AbilityHandAPI
{
   private static final ROS2Topic<AbilityHandLegacyGripCommandMessage> ABILITY_HAND_LEGACY_GRIP_COMMAND_TOPIC = HumanoidControllerAPI.HUMANOID_CONTROLLER.withInput()
                                                                                                                                                         .withTypeName(
                                                                                                                                                               AbilityHandLegacyGripCommandMessage.class);

   public static ROS2Topic<AbilityHandLegacyGripCommandMessage> getAbilityHandLegacyGripCommandTopic(String robotName, RobotSide side)
   {
      return ABILITY_HAND_LEGACY_GRIP_COMMAND_TOPIC.withRobot(robotName).withSuffix(side.getLowerCaseName());
   }
}
