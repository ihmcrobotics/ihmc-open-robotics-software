package us.ihmc.humanoidBehaviors.lookAndStep;

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
   /**
    * Starts the look and step behavior pursuing a goal if not already pursiung a goal.
    * If look and step is already working on a goal, first send a RESET and then send a new GOAL_INPUT. (Todo: Make this better.)
    */
   public static final ROS2Topic<Pose3D> GOAL_INPUT = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Pose3D.class);
   /**
    * Robot will finish taking the current step, the goal will be cleared, and the behavior will wait for a new GOAL_INPUT.
    */
   public static final ROS2Topic<Empty> RESET = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class);
   /**
    * Output that will be send upon reaching the goal.
    */
   public static final ROS2Topic<Empty> REACHED_GOAL = ROS2Tools.BEHAVIOR_MODULE.withOutput().withType(Empty.class);

   /*
    * TODO: Add PAUSE and RESUME that work in any state.
    *  RESET should probably reset more instance variables than just the goal
    */

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
   private static final MessagerAPIFactory.CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

   /*
    * TODO: Review API should contain the data to be reviewed and the Approval should accept a modified version
    */
   public static final MessagerAPIFactory.Topic<Boolean> OperatorReviewEnabled = topic("OperatorReviewEnabled");
   public static final MessagerAPIFactory.Topic<Boolean> ReviewApproval = topic("ReviewApproval");

   // Parameter tuning topics
   public static final MessagerAPIFactory.Topic<List<String>> LookAndStepParameters = topic("LookAndStepParameters");
   public static final MessagerAPIFactory.Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");

   // Visualization only topics
   public static final MessagerAPIFactory.Topic<String> CurrentState = topic("CurrentState");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> StartAndGoalFootPosesForUI = topic("StartAndGoalFootPosesForUI");
   public static final MessagerAPIFactory.Topic<ArrayList<MinimalFootstep>> FootstepPlanForUI = topic("FootstepPlanForUI");
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
