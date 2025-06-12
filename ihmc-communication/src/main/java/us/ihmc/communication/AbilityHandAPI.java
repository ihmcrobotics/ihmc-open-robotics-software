package us.ihmc.communication;

import controller_msgs.msg.dds.AbilityHandCommandMessage;
import controller_msgs.msg.dds.AbilityHandStatusMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;

public final class AbilityHandAPI
{
   public static ROS2Topic<AbilityHandCommandMessage> getHandAbilityCommandTopic(String robotName, RobotSide side)
   {
      return HumanoidControllerAPI.getInputTopic(robotName).withTypeName(AbilityHandCommandMessage.class).withSuffix(side.getLowerCaseName());
   }

   public static ROS2Topic<AbilityHandStatusMessage> getHandAbilityStatusTopic(String robotName, RobotSide side)
   {
      return HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(AbilityHandStatusMessage.class).withSuffix(side.getLowerCaseName());
   }
}
