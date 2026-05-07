package us.ihmc.communication;

import controller_msgs.SakeHandDesiredCommandMessage;
import controller_msgs.SakeHandStatusMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.jros2.ROS2Topic;

public final class SakeHandAPI
{
   public static ROS2Topic<SakeHandDesiredCommandMessage> getHandSakeCommandTopic(String robotName, RobotSide side)
   {
      return HumanoidControllerAPI.getInputTopic(robotName).withType(SakeHandDesiredCommandMessage.class).appendedWith(side.getLowerCaseName());
   }

   public static ROS2Topic<SakeHandStatusMessage> getHandSakeStatusTopic(String robotName, RobotSide side)
   {
      return HumanoidControllerAPI.getOutputTopic(robotName).withType(SakeHandStatusMessage.class).appendedWith(side.getLowerCaseName());
   }
}
