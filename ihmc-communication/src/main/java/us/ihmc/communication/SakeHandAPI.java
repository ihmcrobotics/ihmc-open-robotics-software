package us.ihmc.communication;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;

public final class SakeHandAPI
{
   public static ROS2Topic<SakeHandDesiredCommandMessage> getHandSakeCommandTopic(String robotName, RobotSide side)
   {
      return HumanoidControllerAPI.getInputTopic(robotName).withTypeName(SakeHandDesiredCommandMessage.class).withSuffix(side.getLowerCaseName());
   }

   public static ROS2Topic<SakeHandStatusMessage> getHandSakeStatusTopic(String robotName, RobotSide side)
   {
      return HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(SakeHandStatusMessage.class).withSuffix(side.getLowerCaseName());
   }
}
