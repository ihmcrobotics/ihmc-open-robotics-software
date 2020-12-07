package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StoredPropertySetMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;

public class LookAndStepBehaviorAPI
{
   private static final ROS2Topic<?> LOOK_AND_STEP_BEHAVIOR = ROS2Tools.IHMC_ROOT.withModule(ROS2Tools.BEHAVIOR_MODULE_NAME + "/look_and_step");

   public static final ROS2Topic<PlanarRegionsListMessage> REGIONS_FOR_FOOTSTEP_PLANNING = ROS2Tools.REALSENSE_SLAM_REGIONS;

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
   public static final ROS2Topic<StoredPropertySetMessage> LOOK_AND_STEP_PARAMETERS
         = LOOK_AND_STEP_BEHAVIOR.withType(StoredPropertySetMessage.class).withSuffix("parameters");

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
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> StartAndGoalFootPosesForUI = topic("StartAndGoalFootPosesForUI");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> FootstepPlanForUI = topic("FootstepPlanForUI");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> LastCommandedFootsteps = topic("LastCommandedFootsteps");
   public static final MessagerAPIFactory.Topic<Pose3D> ClosestPointForUI = topic("ClosestPointForUI");
   public static final MessagerAPIFactory.Topic<Pose3D> SubGoalForUI = topic("SubGoalForUI");
   public static final MessagerAPIFactory.Topic<PlanarRegionsList> PlanarRegionsForUI = topic("PlanarRegionsForUI");
   public static final MessagerAPIFactory.Topic<List<Pose3D>> BodyPathPlanForUI = topic("BodyPathPlanForUI");
   public static final MessagerAPIFactory.Topic<Object> ResetForUI = topic("ResetForUI");

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
