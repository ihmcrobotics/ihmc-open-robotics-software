package us.ihmc.footstepPlanning.communication;

import behavior_msgs.ContinuousHikingCommandMessage;
import behavior_msgs.ContinuousWalkingStatusMessage;
import controller_msgs.FootstepDataListMessage;
import ihmc_common_msgs.PoseListMessage;
import perception_msgs.TerrainMapMessage;
import std_msgs.Float32;
import std_msgs.Empty;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.jros2.ROS2Topic;

public class ContinuousHikingAPI
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";
   private static final String moduleName =  "continuous_hiking";
   public static final HumanoidROS2Topic<?> IHMC_ROOT = new HumanoidROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);
   private static final String ACTIVE_MODULE_NAME = "active_perception";

   // Commands supported for the Continuous Hiking Process
   public static final ROS2Topic<Empty> RESET_STATE_MACHINE = IHMC_ROOT.withModule(moduleName).withTypeName(Empty.class).withSuffix("reset_state_machine");
   public static final ROS2Topic<ContinuousHikingCommandMessage> CONTINUOUS_HIKING_COMMAND = IHMC_ROOT.withModule(moduleName).withType(ContinuousHikingCommandMessage.class).withSuffix("command");
   public static final ROS2Topic<Empty> CLEAR_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(Empty.class).withSuffix("clear_goal_footsteps");
   public static final ROS2Topic<PoseListMessage> PLACED_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("placed_goal_footsteps");
   public static final ROS2Topic<PoseListMessage> ROTATE_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("rotate_goal_footsteps");
   public static final ROS2Topic<Float32> ROTATE_90_DEGREES = IHMC_ROOT.withModule(moduleName).withType(Float32.class).withSuffix("rotate_90_degrees");
   public static final ROS2Topic<Empty> SQUARE_UP_STEP = IHMC_ROOT.withModule(moduleName).withType(Empty.class).withSuffix("square_up_step");

   // Statuses supported from the Continuous Hiking Process
   public static final ROS2Topic<ContinuousWalkingStatusMessage> CONTINUOUS_WALKING_STATUS = IHMC_ROOT.withModule(moduleName).withType(ContinuousWalkingStatusMessage.class).withSuffix("status");
   public static final ROS2Topic<FootstepDataListMessage> PLANNED_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(FootstepDataListMessage.class).withSuffix("planned_footsteps");
   public static final ROS2Topic<PoseListMessage> START_AND_GOAL_FOOTSTEPS = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("start_and_goal");
   public static final ROS2Topic<PoseListMessage> MONTE_CARLO_TREE_NODES = IHMC_ROOT.withModule(moduleName).withType(PoseListMessage.class).withSuffix("monte_carlo_tree_nodes");
   public static final ROS2Topic<FootstepDataListMessage> MONTE_CARLO_FOOTSTEP_PLAN = IHMC_ROOT.withModule(moduleName).withType(FootstepDataListMessage.class).withSuffix("monte_carlo_footstep_plan");

   // Parameters that get synced between the remote process and the user
   public static final StoredPropertySetROS2TopicPair PROCESS_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"process_parameters");

   public static final StoredPropertySetROS2TopicPair CONTINUOUS_HIKING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"continuous_hiking_parameters");

   public static final StoredPropertySetROS2TopicPair FOOTSTEP_PLANNING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"footstep_planning_parameters");

   public static final StoredPropertySetROS2TopicPair SWING_PLANNING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"swing_planning_parameters");

   public static final StoredPropertySetROS2TopicPair MONTE_CARLO_PLANNER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"monte_carlo_planner_parameters");

   public static final StoredPropertySetROS2TopicPair DEPTH_IMAGE_FILTERING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"depth_image_filtering_parameters");

   public static final StoredPropertySetROS2TopicPair STEPPABLE_REGION_CALCULATOR_PARAMETERS
         = new StoredPropertySetROS2TopicPair(ACTIVE_MODULE_NAME,"steppable_region_calculator_parameters");
}
