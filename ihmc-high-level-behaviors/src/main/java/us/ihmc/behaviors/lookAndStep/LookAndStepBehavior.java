package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.*;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.ResettingNode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.behaviors.lookAndStep.LookAndStepBodyPathPlanningTask.LookAndStepBodyPathPlanning;
import us.ihmc.behaviors.lookAndStep.LookAndStepFootstepPlanningTask.LookAndStepFootstepPlanning;
import us.ihmc.behaviors.lookAndStep.LookAndStepLocalizationTask.LookAndStepBodyPathLocalization;
import us.ihmc.behaviors.lookAndStep.LookAndStepSteppingTask.LookAndStepStepping;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior extends ResettingNode implements Destroyable
{
   final BehaviorHelper helper;
   final StatusLogger statusLogger;
   final RemoteHumanoidRobotInterface robotInterface;

   final LookAndStepBodyPathPlanning bodyPathPlanning = new LookAndStepBodyPathPlanning();
   final LookAndStepBodyPathLocalization bodyPathLocalization = new LookAndStepBodyPathLocalization();
   final LookAndStepFootstepPlanning footstepPlanning = new LookAndStepFootstepPlanning();
   final LookAndStepStepping stepping = new LookAndStepStepping();
   final LookAndStepReset reset = new LookAndStepReset();
   final BehaviorStateReference<State> behaviorStateReference;
   final ROS2StoredPropertySet<LookAndStepBehaviorParametersBasics> ros2LookAndStepParameters;
   final ROS2StoredPropertySet<DefaultFootstepPlannerParametersBasics> ros2FootstepPlannerParameters;
   final ROS2StoredPropertySet<SwingPlannerParametersBasics> ros2SwingPlannerParameters;
   final PausablePeriodicThread propertyStatusThread;
   final VisibilityGraphsParametersBasics visibilityGraphParameters;
   final AtomicBoolean isBeingReset = new AtomicBoolean();
   final AtomicReference<Boolean> operatorReviewEnabledInput;
   final LookAndStepImminentStanceTracker imminentStanceTracker;
   final ControllerStatusTracker controllerStatusTracker;
   final TypedNotification<Boolean> approvalNotification;

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

      ros2LookAndStepParameters = new ROS2StoredPropertySet<>(helper, PARAMETERS, helper.getRobotModel().getLookAndStepParameters());
      ros2FootstepPlannerParameters = new ROS2StoredPropertySet<>(helper, FOOTSTEP_PLANNING_PARAMETERS,
                                                                  helper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep"));
      ros2SwingPlannerParameters = new ROS2StoredPropertySet<>(helper, SWING_PLANNER_PARAMETERS,
                                                               helper.getRobotModel().getSwingPlannerParameters("ForLookAndStep"));
      propertyStatusThread = new PausablePeriodicThread("PropertyStatusPublisher", ROS2StoredPropertySet.STATUS_PERIOD, () ->
      {
         ros2LookAndStepParameters.updateAndPublishStatus();
         ros2FootstepPlannerParameters.updateAndPublishStatus();
         ros2SwingPlannerParameters.updateAndPublishStatus();
      });
      propertyStatusThread.start();

      operatorReviewEnabledInput = new AtomicReference<>();
      helper.subscribeViaCallback(OPERATOR_REVIEW_ENABLED_COMMAND, enabled ->
      {
         LogTools.info("Received operator review enabled toggle message: {}", enabled.getData());
         operatorReviewEnabledInput.set(enabled.getData());
         helper.publish(OPERATOR_REVIEW_ENABLED_STATUS, enabled.getData());
      });
      approvalNotification = helper.subscribeViaBooleanNotification(REVIEW_APPROVAL);

      // Trying to hold a lot of the state here? TODO: In general, where to put what state?
      imminentStanceTracker = new LookAndStepImminentStanceTracker(helper);
      behaviorStateReference = new BehaviorStateReference<>(State.BODY_PATH_PLANNING, statusLogger, helper);
      controllerStatusTracker = helper.getOrCreateControllerStatusTracker();
      reset.initialize(this);
      helper.subscribeViaCallback(RESET, reset::queueReset);
      helper.subscribeToControllerViaCallback(WalkingControllerFailureStatusMessage.class, message -> reset.queueReset());

      bodyPathPlanning.initialize(this);
      helper.subscribeViaCallback(PerceptionAPI.LIDAR_REA_REGIONS, planarRegionsListMessage ->
      {
         bodyPathPlanning.acceptMapRegions(planarRegionsListMessage);
         footstepPlanning.acceptLidarREARegions(planarRegionsListMessage);
      });
      helper.subscribeViaCallback(GOAL_COMMAND, this::acceptGoal);
      // TODO add height map to footstep planner
      helper.subscribeViaCallback(ROS2_HEIGHT_MAP, bodyPathPlanning::acceptHeightMap);

      bodyPathLocalization.initialize(this);
      helper.subscribeToControllerViaCallback(CapturabilityBasedStatus.class, imminentStanceTracker::acceptCapturabilityBasedStatus);
      helper.subscribeViaCallback(BODY_PATH_INPUT, message -> bodyPathPlanInput(MessageTools.unpackPoseListMessage(message)));

      footstepPlanning.initialize(this);
      helper.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_OUTPUT, heightMapMessage ->
      {
         bodyPathPlanning.acceptHeightMap(heightMapMessage);
         footstepPlanning.acceptHeightMap(heightMapMessage);
      });
      helper.subscribeViaCallback(ROS2_REGIONS_FOR_FOOTSTEP_PLANNING, footstepPlanning::acceptPlanarRegions);
      helper.subscribeViaCallback(StateEstimatorAPI.getRobotConfigurationDataTopic(helper.getRobotModel().getSimpleRobotName()),
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
      helper.publish(GOAL_STATUS, new Pose3D(goal));
      bodyPathLocalization.acceptNewGoalSubmitted();
      bodyPathPlanning.acceptGoal(goal);
   }

   public void setOperatorReviewEnabled(boolean enabled)
   {
      operatorReviewEnabledInput.set(enabled);
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
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
      propertyStatusThread.destroy();
      bodyPathPlanning.destroy();
      footstepPlanning.destroy();
      reset.destroy();
   }

   public String getName()
   {
      return "Look and Step";
   }
}
