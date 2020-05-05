package us.ihmc.footstepPlanning.communication;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class FootstepPlannerCommunicationProperties
{
   public static MessageTopicNameGenerator publisherTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_MODULE_NAME, ROS2TopicQualifier.OUTPUT);
   }

   public static MessageTopicNameGenerator subscriberTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_MODULE_NAME, ROS2TopicQualifier.INPUT);
   }
}
