package us.ihmc.communication;

import controller_msgs.SakeHandDesiredCommandMessage;
import controller_msgs.SakeHandStatusMessage;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotics.robotSide.RobotSide;

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
