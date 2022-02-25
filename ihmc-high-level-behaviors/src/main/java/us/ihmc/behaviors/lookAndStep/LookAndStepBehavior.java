package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.*;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.avatar.sensors.realsense.DelayFixedPlanarRegionsSubscription;
import us.ihmc.behaviors.lookAndStep.LookAndStepBodyPathPlanningTask.LookAndStepBodyPathPlanning;
import us.ihmc.behaviors.lookAndStep.LookAndStepFootstepPlanningTask.LookAndStepFootstepPlanning;
import us.ihmc.behaviors.lookAndStep.LookAndStepLocalizationTask.LookAndStepBodyPathLocalization;
import us.ihmc.behaviors.lookAndStep.LookAndStepSteppingTask.LookAndStepStepping;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   final BehaviorHelper helper;
   final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   final StatusLogger statusLogger;
   final RemoteHumanoidRobotInterface robotInterface;

   final LookAndStepBodyPathPlanning bodyPathPlanning = new LookAndStepBodyPathPlanning();
   final LookAndStepBodyPathLocalization bodyPathLocalization = new LookAndStepBodyPathLocalization();
   final LookAndStepFootstepPlanning footstepPlanning = new LookAndStepFootstepPlanning();
   final LookAndStepStepping stepping = new LookAndStepStepping();
   final LookAndStepReset reset = new LookAndStepReset();
   final BehaviorStateReference<State> behaviorStateReference;
   final LookAndStepBehaviorParameters lookAndStepParameters;
   final FootstepPlannerParametersBasics footstepPlannerParameters;
   final SwingPlannerParametersBasics swingPlannerParameters;
   final VisibilityGraphsParametersBasics visibilityGraphParameters;
   final AtomicBoolean isBeingReset = new AtomicBoolean();
   final AtomicReference<Boolean> operatorReviewEnabledInput;
   final LookAndStepImminentStanceTracker imminentStanceTracker;
   final ControllerStatusTracker controllerStatusTracker;
   final TypedNotification<Boolean> approvalNotification;
   private DelayFixedPlanarRegionsSubscription delayFixedPlanarRegionsSubscription;

   /**
    * At any time the behavior will be executing on one of this tasks
    * or "trying" to do it. Sometimes conditions will not be satisfied
    * to execute the task the behavior will be waiting on those conditions.
    */
   public enum State
   {
      RESET, BODY_PATH_PLANNING, LOCALIZATION, FOOTSTEP_PLANNING, STEPPING;
   }

   /**
    * We'll make the following assumptions/constraints about the data being passed around between task/modules:
    * - Data output from module will be read only and the underlying data must not be modified after sending
    * - Timers are considered parallel tasks/ modules
    *
    * TODO: Want to be able to wire up behavior here and see all present modules.
    * TODO: Add more meaning to the construction by establishing data patterns.
    * TODO: Use named interfaces or pass in actual objects to prevent disordered arguments.
    * TODO: Could aid readability by using local variables before passing into methods.
    *
    * <p>
    * Inputs can be polled or triggered by an event
    * Outputs can be an event or polled by another task
    */
   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");

      robotInterface = helper.getOrCreateRobotInterface();
      statusLogger = helper.getOrCreateStatusLogger();

      visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();
      visibilityGraphParameters.setTooHighToStepDistance(0.2);

      lookAndStepParameters = new LookAndStepBehaviorParameters();
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep");
      swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("ForLookAndStep");

      helper.subscribeViaCallback(LOOK_AND_STEP_PARAMETERS, parameters ->
      {
         List<String> values = Arrays.asList(parameters.getStrings().toStringArray());

         if (!lookAndStepParameters.getAllAsStrings().equals(values))
         {
            statusLogger.info("Accepting new look and step parameters");
            lookAndStepParameters.setAllFromStrings(values);
         }
      });
      helper.subscribeViaCallback(FootstepPlannerParameters, parameters ->
      {
         statusLogger.info("Accepting new footstep planner parameters");
         footstepPlannerParameters.setAllFromStrings(parameters);
      });
      helper.subscribeViaCallback(SwingPlannerParameters, parameters ->
      {
         statusLogger.info("Accepting new swing planner parameters");
         swingPlannerParameters.setAllFromStrings(parameters);
      });

      operatorReviewEnabledInput = new AtomicReference<>();
      helper.subscribeViaCallback(OperatorReviewEnabled, enabled ->
      {
         LogTools.info("Received operator review enabled toggle message: {}", enabled);
         operatorReviewEnabledInput.set(enabled);
         helper.publish(OperatorReviewEnabledToUI, enabled);
      });
      approvalNotification = helper.subscribeViaNotification(ReviewApproval);

      // Trying to hold a lot of the state here? TODO: In general, where to put what state?
      imminentStanceTracker = new LookAndStepImminentStanceTracker(helper);
      behaviorStateReference = new BehaviorStateReference<>(State.BODY_PATH_PLANNING, statusLogger, helper::publish);
      controllerStatusTracker = helper.getOrCreateControllerStatusTracker();
      reset.initialize(this);
      helper.subscribeViaCallback(RESET, reset::queueReset);
      helper.subscribeToControllerViaCallback(WalkingControllerFailureStatusMessage.class, message -> reset.queueReset());

      bodyPathPlanning.initialize(this);
      helper.subscribeViaCallback(ROS2Tools.LIDAR_REA_REGIONS, planarRegionsListMessage ->
      {
         bodyPathPlanning.acceptMapRegions(planarRegionsListMessage);
         footstepPlanning.acceptLidarREARegions(planarRegionsListMessage);
      });
      helper.subscribeViaCallback(GOAL_INPUT, this::acceptGoal);

      bodyPathLocalization.initialize(this);
      helper.subscribeToControllerViaCallback(CapturabilityBasedStatus.class, imminentStanceTracker::acceptCapturabilityBasedStatus);
      helper.subscribeViaCallback(BodyPathInput, this::bodyPathPlanInput);

      footstepPlanning.initialize(this);
      if (helper.getROS1Helper() != null)
      {
         delayFixedPlanarRegionsSubscription = helper.subscribeToPlanarRegionsViaCallback(REGIONS_FOR_FOOTSTEP_PLANNING, regions -> footstepPlanning.acceptPlanarRegions(regions.getRight()));
         delayFixedPlanarRegionsSubscription.subscribe(helper.getROS1Helper());
         delayFixedPlanarRegionsSubscription.setEnabled(true);
         delayFixedPlanarRegionsSubscription.setPosePublisherEnabled(true);
      }
      helper.subscribeViaCallback(ROS2_REGIONS_FOR_FOOTSTEP_PLANNING, footstepPlanning::acceptPlanarRegions);
      helper.subscribeViaCallback(ROS2Tools.getRobotConfigurationDataTopic(helper.getRobotModel().getSimpleRobotName()),
                                  footstepPlanning::acceptRobotConfigurationData);
      helper.subscribeToControllerViaCallback(CapturabilityBasedStatus.class, footstepPlanning::acceptCapturabilityBasedStatus);
      helper.subscribeToControllerViaCallback(FootstepStatusMessage.class, status ->
      {
         if (status.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            footstepPlanning.acceptFootstepCompleted();
         }
         else
         {
            footstepPlanning.acceptFootstepStarted(status);
         }
      });

      stepping.initialize(this);

      reset.queueReset();
   }

   void bodyPathPlanInput(List<? extends Pose3DReadOnly> bodyPath)
   {
      if (!isBeingReset.get())
      {
         behaviorStateReference.set(LookAndStepBehavior.State.LOCALIZATION);
         bodyPathLocalization.acceptBodyPathPlan(bodyPath);
      }
   }

   public void acceptGoal(Pose3DReadOnly goal)
   {
      behaviorStateReference.broadcast();
      helper.publish(GoalForUI, new Pose3D(goal));
      bodyPathLocalization.acceptNewGoalSubmitted();
      bodyPathPlanning.acceptGoal(goal);
   }

   public void setOperatorReviewEnabled(boolean enabled)
   {
      operatorReviewEnabledInput.set(enabled);
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return BehaviorTreeNodeStatus.RUNNING;
   }

   @Override
   public void reset()
   {
      reset.queueReset();
   }

   public void destroy()
   {
      if (delayFixedPlanarRegionsSubscription != null)
         delayFixedPlanarRegionsSubscription.destroy();
      bodyPathPlanning.destroy();
      footstepPlanning.destroy();
      reset.destroy();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
