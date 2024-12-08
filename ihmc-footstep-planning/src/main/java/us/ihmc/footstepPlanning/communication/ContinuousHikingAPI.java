package us.ihmc.footstepPlanning.communication;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import behavior_msgs.msg.dds.ContinuousWalkingStatusMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.ros2.ROS2Topic;

public class ContinuousHikingAPI
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";
   private static final String moduleName =  "continuous_hiking";
   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);
   private static final String ACTIVE_MODULE_NAME = "active_perception";

   // Commands supported for the Continuous Hiking Process
   public static final ROS2Topic<ContinuousHikingCommandMessage> CONTINUOUS_HIKING_COMMAND = IHMC_ROOT.withModule(moduleName).withType(ContinuousHikingCommandMessage.class).withSuffix("command");
   public static final ROS2Topic<std_msgs.msg.dds.Empty> CLEAR_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(std_msgs.msg.dds.Empty.class).withSuffix("clear_goal_footsteps");
   public static final ROS2Topic<PoseListMessage> PLACED_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("placed_goal_footsteps");

   // Statuses supported from the Continuous Hiking Process
   public static final ROS2Topic<ContinuousWalkingStatusMessage> CONTINUOUS_WALKING_STATUS = IHMC_ROOT.withModule(moduleName).withType(ContinuousWalkingStatusMessage.class).withSuffix("status");
   public static final ROS2Topic<FootstepDataListMessage> PLANNED_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(FootstepDataListMessage.class).withSuffix("planned_footsteps");
   public static final ROS2Topic<PoseListMessage> START_AND_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("start_and_goal");
   public static final ROS2Topic<PoseListMessage> MONTE_CARLO_TREE_NODES = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("monte_carlo_tree_nodes");
   public static final ROS2Topic<FootstepDataListMessage> MONTE_CARLO_FOOTSTEP_PLAN = IHMC_ROOT.withModule(moduleName).withType(FootstepDataListMessage.class).withSuffix("monte_carlo_footstep_plan");

   // Parameters that get synced between the remote process and the user
   public static final StoredPropertySetROS2TopicPair CONTINUOUS_HIKING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"continuous_hiking_parameters");

   public static final StoredPropertySetROS2TopicPair FOOTSTEP_PLANNING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"footstep_planning_parameters");

   public static final StoredPropertySetROS2TopicPair SWING_PLANNING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"swing_planning_parameters");

   public static final StoredPropertySetROS2TopicPair MONTE_CARLO_PLANNER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"monte_carlo_planner_parameters");
}
