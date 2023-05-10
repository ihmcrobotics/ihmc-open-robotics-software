package us.ihmc.behaviors.lookAndStep;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
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
   private static final ROS2Topic<?> LOOK_AND_STEP_BEHAVIOR = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);

   public static final String REGIONS_FOR_FOOTSTEP_PLANNING = RosTools.MAPSENSE_REGIONS;
   public static final ROS2Topic<PlanarRegionsListMessage> ROS2_REGIONS_FOR_FOOTSTEP_PLANNING = PerceptionAPI.PERSPECTIVE_RAPID_REGIONS;
   public static final ROS2Topic<HeightMapMessage> ROS2_HEIGHT_MAP = PerceptionAPI.HEIGHT_MAP_OUTPUT;

   /**
    * Starts the look and step behavior pursuing a goal if not already pursiung a goal.
    * If look and step is already working on a goal, first send a RESET and then send a new GOAL_INPUT. (Todo: Make this better.)
    */
   public static final ROS2Topic<Pose3D> GOAL_INPUT = LOOK_AND_STEP_BEHAVIOR.withInput().withTypeName(Pose3D.class);
   /**
    * Robot will finish taking the current step, the goal will be cleared, and the behavior will wait for a new GOAL_INPUT.
    */
   public static final ROS2Topic<Empty> RESET = LOOK_AND_STEP_BEHAVIOR.withInput().withTypeName(Empty.class);
   /**
    * Output that will be send upon reaching the goal.
    */
   public static final ROS2Topic<Empty> REACHED_GOAL = LOOK_AND_STEP_BEHAVIOR.withOutput().withTypeName(Empty.class);
   /** Look and step behavior parameters */
   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "parameters");
   public static final StoredPropertySetROS2TopicPair FOOTSTEP_PLANNING_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME,
                                                                                                                        "footstep_planning_parameters");
   public static final StoredPropertySetROS2TopicPair SWING_PLANNER_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "swing_planner_parameters");
   public static final ROS2Topic<HeightMapMessage> HEIGHT_MAP_FOR_UI = LOOK_AND_STEP_BEHAVIOR.withType(HeightMapMessage.class).withSuffix("height_map_for_ui");

   /*
    * TODO: Add PAUSE and RESUME that work in any state.
    *  RESET should probably reset more instance variables than just the goal
    */

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
   private static final MessagerAPIFactory.CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

   /** Starts the look and step behavior onto a precomputed body path */
   public static final MessagerAPIFactory.Topic<List<Pose3D>> BodyPathInput = topic("BodyPathInput");

   /*
    * TODO: Review API should contain the data to be reviewed and the Approval should accept a modified version
    */
   public static final MessagerAPIFactory.Topic<Object> PublishSupportRegions = topic("PublishSupportRegions");
   public static final MessagerAPIFactory.Topic<Boolean> OperatorReviewEnabled = topic("OperatorReviewEnabled");
   public static final MessagerAPIFactory.Topic<Boolean> OperatorReviewEnabledToUI = topic("OperatorReviewEnabledToUI");
   public static final MessagerAPIFactory.Topic<Boolean> ReviewApproval = topic("ReviewApproval");

   // Parameter tuning topics
   public static final MessagerAPIFactory.Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");
   public static final MessagerAPIFactory.Topic<List<String>> SwingPlannerParameters = topic("SwingPlannerParameters");

   // Visualization only topics
   public static final MessagerAPIFactory.Topic<String> CurrentState = topic("CurrentState");
   public static final MessagerAPIFactory.Topic<Boolean> ToggleAllVisualization = topic("ToggleAllVisualization");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> ImminentFootPosesForUI = topic("ImminentFootPosesForUI");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> PlannedFootstepsForUI = topic("PlannedFootstepsForUI");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> LastCommandedFootsteps = topic("LastCommandedFootsteps");
   public static final MessagerAPIFactory.Topic<Pose3D> ClosestPointForUI = topic("ClosestPointForUI");
   public static final MessagerAPIFactory.Topic<Pose3D> GoalForUI = topic("GoalForUI");
   public static final MessagerAPIFactory.Topic<Pose3D> SubGoalForUI = topic("SubGoalForUI");
   public static final MessagerAPIFactory.Topic<PlanarRegionsList> PlanarRegionsForUI = topic("PlanarRegionsForUI");
   public static final MessagerAPIFactory.Topic<PlanarRegionsList> ReceivedPlanarRegionsForUI = topic("ReceivedlanarRegionsForUI");
   public static final MessagerAPIFactory.Topic<Boolean> ImpassibilityDetected = topic("ImpassibilityDetected");
   public static final MessagerAPIFactory.Topic<Boolean> PlanningFailed = topic("PlanningFailed");
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
