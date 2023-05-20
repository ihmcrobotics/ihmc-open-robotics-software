package us.ihmc.behaviors.lookAndStep;

import behavior_msgs.msg.dds.MinimalFootstepListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.utilities.ros.RosTools;

import java.util.ArrayList;
import java.util.List;

public class LookAndStepBehaviorAPI
{
   private static final String MODULE_NAME = ROS2Tools.BEHAVIOR_MODULE_NAME + "/look_and_step";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);

   public static final String REGIONS_FOR_FOOTSTEP_PLANNING = RosTools.MAPSENSE_REGIONS;
   public static final ROS2Topic<PlanarRegionsListMessage> ROS2_REGIONS_FOR_FOOTSTEP_PLANNING = PerceptionAPI.PERSPECTIVE_RAPID_REGIONS;
   public static final ROS2Topic<HeightMapMessage> ROS2_HEIGHT_MAP = PerceptionAPI.HEIGHT_MAP_OUTPUT;

   /**
    * Starts the look and step behavior pursuing a goal if not already pursiung a goal.
    * If look and step is already working on a goal, first send a RESET and then send a new GOAL_INPUT. (Todo: Make this better.)
    */
   public static final ROS2IOTopicPair<Pose3D> GOAL = new ROS2IOTopicPair<>(BASE_TOPIC.withType(Pose3D.class).withSuffix("goal"));
   public static final ROS2Topic<Pose3D> GOAL_COMMAND = GOAL.getCommandTopic();
   public static final ROS2Topic<Pose3D> GOAL_STATUS = GOAL.getStatusTopic();
   /**
    * Robot will finish taking the current step, the goal will be cleared, and the behavior will wait for a new GOAL_INPUT.
    */
   public static final ROS2Topic<Empty> RESET = BASE_TOPIC.withInput().withTypeName(Empty.class);
   /**
    * Output that will be send upon reaching the goal.
    */
   public static final ROS2Topic<Empty> REACHED_GOAL = BASE_TOPIC.withOutput().withTypeName(Empty.class);
   /** Look and step behavior parameters */
   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "parameters");
   public static final StoredPropertySetROS2TopicPair FOOTSTEP_PLANNING_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME,
                                                                                                                        "footstep_planning_parameters");
   public static final StoredPropertySetROS2TopicPair SWING_PLANNER_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "swing_planner_parameters");
   public static final ROS2Topic<HeightMapMessage> HEIGHT_MAP_FOR_UI = BASE_TOPIC.withType(HeightMapMessage.class).withSuffix("height_map_for_ui");

   /*
    * TODO: Add PAUSE and RESUME that work in any state.
    *  RESET should probably reset more instance variables than just the goal
    */

   /** Starts the look and step behavior onto a precomputed body path */
   public static final ROS2Topic<PoseListMessage> BODY_PATH_INPUT = BASE_TOPIC.withType(PoseListMessage.class).withInput().withSuffix("body_path");

   /*
    * TODO: Review API should contain the data to be reviewed and the Approval should accept a modified version
    */
   public static final ROS2IOTopicPair<Bool> OPERATOR_REVIEW_ENABLED = new ROS2IOTopicPair<>(BASE_TOPIC.withType(Bool.class)
                                                                                                       .withSuffix("operator_review_enabled"));
   public static final ROS2Topic<Bool> OPERATOR_REVIEW_ENABLED_COMMAND = OPERATOR_REVIEW_ENABLED.getCommandTopic();
   public static final ROS2Topic<Bool> OPERATOR_REVIEW_ENABLED_STATUS = OPERATOR_REVIEW_ENABLED.getStatusTopic();
   public static final ROS2Topic<Bool> REVIEW_APPROVAL = BASE_TOPIC.withType(Bool.class).withSuffix("review_approval");

   // Visualization only topics
   public static final ROS2Topic<std_msgs.msg.dds.String> CURRENT_STATE = BASE_TOPIC.withType(std_msgs.msg.dds.String.class).withSuffix("current_state");
   public static final ROS2Topic<MinimalFootstepListMessage> IMMINENT_FOOT_POSES_FOR_UI = BASE_TOPIC.withType(MinimalFootstepListMessage.class)
                                                                                                    .withSuffix("imminent_foot_poses_for_ui");
   public static final ROS2Topic<MinimalFootstepListMessage> PLANNED_FOOTSTEPS_FOR_UI = BASE_TOPIC.withType(MinimalFootstepListMessage.class)
                                                                                                  .withSuffix("planned_footsteps_for_ui");
   public static final ROS2Topic<MinimalFootstepListMessage> LAST_COMMANDED_FOOTSTEPS = BASE_TOPIC.withType(MinimalFootstepListMessage.class)
                                                                                                  .withSuffix("last_commanded_footsteps");
   public static final ROS2Topic<Pose3D> CLOSEST_POINT_FOR_UI = BASE_TOPIC.withType(Pose3D.class).withSuffix("closest_point_for_ui");
   public static final ROS2Topic<Pose3D> SUB_GOAL_FOR_UI = BASE_TOPIC.withType(Pose3D.class).withSuffix("sub_goal_for_ui");
   /**
    * The planar regions that are being used for footstep planning.
    * These are important to inspect the regions used, to figure out why planning went wrong
    * or something.
    */
   public static final ROS2Topic<PlanarRegionsListMessage> PLANAR_REGIONS_FOR_UI = BASE_TOPIC.withType(PlanarRegionsListMessage.class)
                                                                                             .withSuffix("planar_regions_for_ui");
   /**
    * The latest set of planar regions received by the behavior; regardless if being used.
    * This topic is useful to just see if the behavior is getting regions data and see
    * approximately the rate, quality, and jitter of them.
    */
   public static final ROS2Topic<PlanarRegionsListMessage> RECEIVED_PLANAR_REGIONS_FOR_UI = BASE_TOPIC.withType(PlanarRegionsListMessage.class)
                                                                                                      .withSuffix("received_planar_regions_for_ui");
   public static final ROS2Topic<Bool> IMPASSIBILITY_DETECTED = BASE_TOPIC.withType(Bool.class).withSuffix("impassibility_detected");
   public static final ROS2Topic<Bool> PLANNING_FAILED = BASE_TOPIC.withType(Bool.class).withSuffix("planning_failed");

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
   private static final MessagerAPIFactory.CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

   // Visualization only topics
   public static final MessagerAPIFactory.Topic<MutablePair<Pose3D, Vector3D>> Obstacle = topic("Obstacle");
   public static final MessagerAPIFactory.Topic<List<Pose3D>> BodyPathPlanForUI = topic("BodyPathPlanForUI");
   public static final MessagerAPIFactory.Topic<Object> ResetForUI = topic("ResetForUI");
   public static final MessagerAPIFactory.Topic<Double> MeasuredPlanarRegionDelay = topic("MeasuredPlanarRegionDelay");
   public static final MessagerAPIFactory.Topic<ArrayList<Pair<Integer, Double>>> FootstepPlannerRejectionReasons = topic("FootstepPlannerRejectionReasons");
   public static final MessagerAPIFactory.Topic<String> FootstepPlannerLatestLogPath = topic("FootstepPlannerLatestLogPath");

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
