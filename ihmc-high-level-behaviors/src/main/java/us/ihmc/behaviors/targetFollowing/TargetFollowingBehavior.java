package us.ihmc.behaviors.targetFollowing;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI.*;
import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorTools.NAN_POSE;
import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.OperatorReviewEnabled;
import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.OperatorReviewEnabledToUI;

public class TargetFollowingBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Target Following", TargetFollowingBehavior::new, TargetFollowingBehaviorAPI.API);

   private final BehaviorHelper helper;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final TargetFollowingBehaviorParameters parameters;
   private final ROS2SyncedRobotModel syncedRobot;
   private final AtomicReference<Pose3D> goal = new AtomicReference<>(NAN_POSE);
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("CommsRelay", true);
   private final ControllerStatusTracker controllerStatusTracker;

   private String lastTickedThing = "NONE";
   private int lastNumberOfCompletedFoosteps = 0;

   public TargetFollowingBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      parameters = new TargetFollowingBehaviorParameters();
      syncedRobot = helper.newSyncedRobot();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      addChild(lookAndStepBehavior);
      helper.subscribeViaCallback(Parameters, parameters ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new building exploration parameters");
         this.parameters.setAllFromStrings(parameters);
      });
      helper.subscribeViaCallback(Goal, newGoal ->
      {
         goal.set(newGoal);
      });
      helper.subscribeViaCallback(Mode, newValue ->
      {
         LogTools.info("Received mode: {}", newValue);
      });
      controllerStatusTracker = helper.getOrCreateControllerStatusTracker();
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      // if look and step took a step since we last looked
      int numberOfCompletedFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfCompletedFootsteps();
      int newlyCompletedFootsteps = numberOfCompletedFootsteps - lastNumberOfCompletedFoosteps;
      lastNumberOfCompletedFoosteps = numberOfCompletedFootsteps;

      helper.publish(OperatorReviewEnabled, false);
      helper.publish(OperatorReviewEnabledToUI, false);

      if (newlyCompletedFootsteps > 0 || lookAndStepBehavior.isReset())
      {
//         lookAndStepBehavior.setOperatorReviewEnabled(false);
         lookAndStepBehavior.acceptGoal(goal.get());
      }

      lastTickedThing = "LOOK_AND_STEP";
      return lookAndStepBehavior.tick();
   }

   @Override
   public void reset()
   {

   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }

   @Override
   public void destroy()
   {
      lookAndStepBehavior.destroy();
   }
}


