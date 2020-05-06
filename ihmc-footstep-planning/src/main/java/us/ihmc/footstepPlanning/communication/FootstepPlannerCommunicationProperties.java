package us.ihmc.footstepPlanning.communication;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2MessageTopicNameGenerator;
import us.ihmc.ros2.ROS2TopicQualifier;

public class FootstepPlannerCommunicationProperties
{
   public static ROS2MessageTopicNameGenerator publisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.IHMC_ROOT.robot(robotName).module(ROS2Tools.FOOTSTEP_PLANNER_MODULE_NAME).qualifier(ROS2TopicQualifier.OUTPUT);
   }

   public static ROS2MessageTopicNameGenerator subscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.IHMC_ROOT.robot(robotName).module(ROS2Tools.FOOTSTEP_PLANNER_MODULE_NAME).qualifier(ROS2TopicQualifier.INPUT);
   }
}
