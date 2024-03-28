package us.ihmc.communication;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.ros2.ROS2Topic;

public class FootstepPlannerAPI
{
   public static final String FOOTSTEP_PLANNER_NODE_NAME = "ihmc_multi_stage_footstep_planning_module";
   public static final String FOOTSTEP_PLANNER_MODULE_NAME = "toolbox/footstep_plan";

   public static final ROS2Topic<?> FOOTSTEP_PLANNER = ROS2Tools.IHMC_ROOT.withModule(FOOTSTEP_PLANNER_MODULE_NAME);

   /** By default, topics are made using "typed topic names" from this base topic */
   public static ROS2Topic<?> outputTopic(String robotName)
   {
      return FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
   }

   /** By default, topics are made using "typed topic names" from this base topic */
   public static ROS2Topic<?> inputTopic(String robotName)
   {
      return FOOTSTEP_PLANNER.withRobot(robotName).withInput();
   }

   public static ROS2Topic<FootstepDataListMessage> swingReplanOutputTopic(String robotName)
   {
      return outputTopic(robotName).withInput().withType(FootstepDataListMessage.class).withSuffix("/replan_swing");
   }
}
