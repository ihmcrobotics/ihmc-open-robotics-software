package us.ihmc.footstepPlanning.communication;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2TopicName;

public class FootstepPlannerCommunicationProperties
{
   public static ROS2TopicName outputTopicName(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
   }

   public static ROS2TopicName subscriberTopicName(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
   }
}
