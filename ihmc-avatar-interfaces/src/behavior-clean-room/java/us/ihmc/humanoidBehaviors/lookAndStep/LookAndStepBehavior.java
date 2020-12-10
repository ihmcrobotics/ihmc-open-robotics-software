package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBodyPathPlanningTask.LookAndStepBodyPathPlanning;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepFootstepPlanningTask.LookAndStepFootstepPlanning;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepLocalizationTask.LookAndStepBodyPathLocalization;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepSteppingTask.LookAndStepStepping;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   final BehaviorHelper helper;
   final StatusLogger statusLogger;
   final RemoteHumanoidRobotInterface robotInterface;

   final LookAndStepBodyPathPlanning bodyPathPlanning = new LookAndStepBodyPathPlanning();
   final LookAndStepBodyPathLocalization bodyPathLocalization = new LookAndStepBodyPathLocalization();
   final LookAndStepFootstepPlanning footstepPlanning = new LookAndStepFootstepPlanning();
   final LookAndStepStepping stepping = new LookAndStepStepping();
   final LookAndStepReset reset = new LookAndStepReset();
   final LookAndStepSupportRegionsPublisher supportRegionsPublisher = new LookAndStepSupportRegionsPublisher();
   final BehaviorStateReference<State> behaviorStateReference;
   final LookAndStepBehaviorParameters lookAndStepParameters;
   final FootstepPlannerParametersBasics footstepPlannerParameters;
   final SwingPlannerParametersBasics swingPlannerParameters;
   final VisibilityGraphsParametersBasics visibilityGraphParameters;
   final AtomicBoolean isBeingReset = new AtomicBoolean();
   final AtomicReference<Boolean> operatorReviewEnabledInput;
   final AtomicReference<RobotSide> lastStanceSide;
   final SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps;
   final ControllerStatusTracker controllerStatusTracker;
   final TypedNotification<Boolean> approvalNotification;

   /**
    * At any time the behavior will be executing on one of this tasks
    * or "trying" to do it. Sometimes conditions will not be satisfied
    * to execute the task the behavior will be waiting on those conditions.
    */
   public enum State
   {
      RESET, BODY_PATH_PLANNING, FOOTSTEP_PLANNING, STEPPING;
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

      robotInterface = helper.getOrCreateRobotInterface();
      statusLogger = helper.getOrCreateStatusLogger();

      visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();
      visibilityGraphParameters.setIncludePreferredExtrusions(false);
      visibilityGraphParameters.setTooHighToStepDistance(0.2);

      lookAndStepParameters = new LookAndStepBehaviorParameters();
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep");
      swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("ForLookAndStep");

      helper.createROS2Callback(LOOK_AND_STEP_PARAMETERS, parameters ->
      {
         List<String> values = Arrays.asList(parameters.getStrings().toStringArray());

         if (!lookAndStepParameters.getAllAsStrings().equals(values))
         {
            statusLogger.info("Accepting new look and step parameters");
            lookAndStepParameters.setAllFromStrings(values);
         }
      });
      helper.createUICallback(FootstepPlannerParameters, parameters ->
      {
         statusLogger.info("Accepting new footstep planner parameters");
         footstepPlannerParameters.setAllFromStrings(parameters);
      });
      helper.createUICallback(SwingPlannerParameters, parameters ->
      {
         statusLogger.info("Accepting new swing planner parameters");
         swingPlannerParameters.setAllFromStrings(parameters);
      });

      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      approvalNotification = helper.createUITypedNotification(ReviewApproval);

      // Trying to hold a lot of the state here? TODO: In general, where to put what state?
      lastStanceSide = new AtomicReference<>();
      lastCommandedFootsteps = new SideDependentList<>();
      behaviorStateReference = new BehaviorStateReference<>(State.BODY_PATH_PLANNING, statusLogger, helper::publishToUI);
      controllerStatusTracker = new ControllerStatusTracker(statusLogger,
                                                            helper.getManagedROS2Node(),
                                                            helper.getRobotModel().getSimpleRobotName());
      reset.initialize(this);
      helper.createROS2Callback(RESET, reset::queueReset);
      helper.createROS2ControllerCallback(WalkingControllerFailureStatusMessage.class, message -> reset.queueReset());

      supportRegionsPublisher.initialize(statusLogger, lookAndStepParameters, helper::publishROS2);
      helper.createROS2Callback(REGIONS_FOR_FOOTSTEP_PLANNING, supportRegionsPublisher::acceptPlanarRegions);
      helper.createROS2ControllerCallback(CapturabilityBasedStatus.class, supportRegionsPublisher::acceptCapturabilityBasedStatus);
      helper.createUICallback(PublishSupportRegions, message -> supportRegionsPublisher.queuePublish());

      bodyPathPlanning.initialize(this);
      helper.createROS2Callback(ROS2Tools.LIDAR_REA_REGIONS, bodyPathPlanning::acceptMapRegions);
      helper.createROS2Callback(GOAL_INPUT, bodyPathPlanning::acceptGoal);

      bodyPathLocalization.initialize(this);
      helper.createROS2ControllerCallback(CapturabilityBasedStatus.class, bodyPathLocalization::acceptCapturabilityBasedStatus);
      helper.createUICallback(BodyPathInput, this::bodyPathPlanInput);

      footstepPlanning.initialize(this);
      helper.createROS2Callback(REGIONS_FOR_FOOTSTEP_PLANNING, footstepPlanning::acceptPlanarRegions);
      helper.createROS2ControllerCallback(CapturabilityBasedStatus.class, footstepPlanning::acceptCapturabilityBasedStatus);
      helper.createROS2ControllerCallback(FootstepStatusMessage.class, status ->
      {
         if (status.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            footstepPlanning.acceptFootstepCompleted();
         }
      });

      stepping.initialize(this);
   }

   void bodyPathPlanInput(List<? extends Pose3DReadOnly> bodyPath)
   {
      if (!isBeingReset.get())
      {
         behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
         bodyPathLocalization.acceptBodyPathPlan(bodyPath);
      }
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      helper.setCommunicationCallbacksEnabled(enabled);
      behaviorStateReference.broadcast();
      reset.queueReset();
   }
}
