package us.ihmc.communication;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;

public class SakeHandAPI
{
   private static final ROS2Topic<SakeHandDesiredCommandMessage> HAND_SAKE_DESIRED_COMMAND
         = HumanoidControllerAPI.HUMANOID_CONTROLLER.withInput().withTypeName(SakeHandDesiredCommandMessage.class);
   private static final ROS2Topic<SakeHandStatusMessage> HAND_SAKE_DESIRED_STATUS
         = HumanoidControllerAPI.HUMANOID_CONTROLLER.withOutput().withTypeName(SakeHandStatusMessage.class);

   public static ROS2Topic<SakeHandDesiredCommandMessage> getHandSakeCommandTopic(String robotName, RobotSide side)
   {
      return HAND_SAKE_DESIRED_COMMAND.withRobot(robotName).withSuffix(side.getLowerCaseName());
   }

   public static ROS2Topic<SakeHandStatusMessage> getHandSakeStatusTopic(String robotName, RobotSide side)
   {
      return HAND_SAKE_DESIRED_STATUS.withRobot(robotName).withSuffix(side.getLowerCaseName());
   }
}
