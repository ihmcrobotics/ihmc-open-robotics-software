package us.ihmc.footstepPlanning.communication;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2MessageTopicNameGenerator;
import us.ihmc.ros2.ROS2TopicQualifier;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class FootstepPlannerCommunicationProperties
{
   public static ROS2MessageTopicNameGenerator publisherTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_MODULE_NAME, ROS2TopicQualifier.OUTPUT);
   }

   public static ROS2MessageTopicNameGenerator subscriberTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_MODULE_NAME, ROS2TopicQualifier.INPUT);
   }
}
