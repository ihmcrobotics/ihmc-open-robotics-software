package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class LookAndStepBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
   private static final MessagerAPIFactory.CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

   public static final MessagerAPIFactory.Topic<String> CurrentState = topic("CurrentState");
   public static final MessagerAPIFactory.Topic<String> StatusLog = topic("StatusLog");
   public static final MessagerAPIFactory.Topic<Object> TakeStep = topic("TakeStep"); // TODO remove?
   public static final MessagerAPIFactory.Topic<Object> RePlan = topic("RePlan"); // TODO remove?
   public static final MessagerAPIFactory.Topic<Boolean> Approval = topic("Approval");
   public static final MessagerAPIFactory.Topic<Boolean> OperatorReviewEnabled = topic("OperatorReview");
   public static final MessagerAPIFactory.Topic<Boolean> AbortGoalWalking = topic("AbortGoalWalking");
   public static final MessagerAPIFactory.Topic<ArrayList<FootstepForUI>> StartAndGoalFootPosesForUI = topic("StartAndGoalFootPosesForUI");
   public static final MessagerAPIFactory.Topic<ArrayList<FootstepForUI>> FootstepPlanForUI = topic("FootstepPlanForUI");
   public static final MessagerAPIFactory.Topic<Pose3D> ClosestPointForUI = topic("ClosestPointForUI");
   public static final MessagerAPIFactory.Topic<Pose3D> SubGoalForUI = topic("SubGoalForUI");
   public static final MessagerAPIFactory.Topic<PlanarRegionsList> MapRegionsForUI = topic("MapRegionsForUI");
   public static final MessagerAPIFactory.Topic<List<String>> LookAndStepParameters = topic("LookAndStepParameters");
   public static final MessagerAPIFactory.Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");
   public static final MessagerAPIFactory.Topic<Pose3D> GoalInput = topic("GoalInput");
   public static final MessagerAPIFactory.Topic<List<Pose3D>> BodyPathPlanForUI = topic("BodyPathPlanForUI");

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
