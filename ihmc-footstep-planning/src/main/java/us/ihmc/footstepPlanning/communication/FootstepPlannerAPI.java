package us.ihmc.footstepPlanning.communication;

import controller_msgs.msg.dds.FootstepDataListMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class FootstepPlannerAPI
{
   /** By default, topics are made using "typed topic names" from this base topic */
   public static ROS2Topic outputTopic(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
   }

   /** By default, topics are made using "typed topic names" from this base topic */
   public static ROS2Topic inputTopic(String robotName)
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
   }

   public static ROS2Topic<Byte> swingReplanRequestTopic(String robotName)
   {
      return inputTopic(robotName).withInput().withType(Byte.class).withSuffix("/replan_swing");
   }

   public static ROS2Topic<FootstepDataListMessage> swingReplanOutputTopic(String robotName)
   {
      return outputTopic(robotName).withInput().withType(FootstepDataListMessage.class).withSuffix("/replan_swing");
   }
}
