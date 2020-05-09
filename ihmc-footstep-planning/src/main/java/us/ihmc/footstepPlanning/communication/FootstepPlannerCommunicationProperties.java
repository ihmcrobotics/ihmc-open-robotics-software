package us.ihmc.footstepPlanning.communication;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class FootstepPlannerCommunicationProperties
{
   public static ROS2Topic outputTopicName(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic inputTopicName(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
   }
}
