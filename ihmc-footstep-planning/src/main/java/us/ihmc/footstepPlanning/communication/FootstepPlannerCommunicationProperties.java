package us.ihmc.footstepPlanning.communication;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2TopicName;

public class FootstepPlannerCommunicationProperties
{
   public static ROS2TopicName publisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.robot(robotName).suffix(ROS2Tools.OUTPUT);
   }

   public static ROS2TopicName subscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.robot(robotName).suffix(ROS2Tools.INPUT);
   }
}
