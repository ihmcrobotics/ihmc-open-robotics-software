package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBodyPathPlanningTask.LookAndStepBodyPathPlanning;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepFootstepPlanningTask.LookAndStepFootstepPlanning;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepLocalizationTask.LookAndStepLocalization;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepSteppingTask.LookAndStepStepping;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.ros2.ROS2TypelessInput;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   private final BehaviorHelper helper;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;

   private final LookAndStepBodyPathPlanning bodyPathPlanning = new LookAndStepBodyPathPlanning();
   private final LookAndStepLocalization localization = new LookAndStepLocalization();
   private final LookAndStepFootstepPlanning footstepPlanning = new LookAndStepFootstepPlanning();
   private final LookAndStepStepping stepping = new LookAndStepStepping();
   private final LookAndStepReset reset = new LookAndStepReset();
   private final BehaviorStateReference<State> behaviorStateReference;
   private final LookAndStepBehaviorParameters lookAndStepParameters;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;

   /**
    * At any time the behavior will be executing on one of this tasks
    * or "trying" to do it. Sometimes conditions will not be satisfied
    * to execute the task the behavior will be waiting on those conditions.
    */
   public enum State
   {
      BODY_PATH_PLANNING, FOOTSTEP_PLANNING, STEPPING;
   }

   /**
    * We'll make the following assumptions/constraints about the data being passed around between task/modules:
    * - Data output from module will be read only and the underlying data must not be modified after sending
    * - Timers are considered parallel tasks/ modules
    * <p>
    * Inputs can be polled or triggered by an event
    * Outputs can be an event or polled by another task
    */
   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;

      robotInterface = helper.getOrCreateRobotInterface();
      statusLogger = helper.getOrCreateStatusLogger();

      VisibilityGraphsParametersBasics visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();
      visibilityGraphParameters.setIncludePreferredExtrusions(false);
      visibilityGraphParameters.setTooHighToStepDistance(0.2);

      lookAndStepParameters = new LookAndStepBehaviorParameters();
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters();

      updateOverrideParameters();

      helper.createUICallback(LookAndStepParameters, parameters ->
      {
         statusLogger.info("Accepting new look and step parameters");
         lookAndStepParameters.setAllFromStrings(parameters);
         updateOverrideParameters();
      });
      helper.createUICallback(FootstepPlannerParameters, parameters ->
      {
         statusLogger.info("Accepting new footstep planner parameters");
         footstepPlannerParameters.setAllFromStrings(parameters);
      }); // TODO: This overrides overrides?

      AtomicReference<Boolean> operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      TypedNotification<Boolean> approvalNotification = helper.createUITypedNotification(ReviewApproval);

      // Trying to hold a lot of the state here? TODO: In general, where to put what state?
      AtomicReference<RobotSide> lastStanceSide = new AtomicReference<>();
      SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps = new SideDependentList<>();
      behaviorStateReference = new BehaviorStateReference<>(State.BODY_PATH_PLANNING, statusLogger, helper::publishToUI);
      AtomicBoolean isBeingReset = new AtomicBoolean();
      ROS2TypelessInput resetInput = helper.createROS2TypelessInput(RESET);
      ControllerStatusTracker controllerStatusTracker = new ControllerStatusTracker(statusLogger,
                                                                                   helper.getManagedROS2Node(),
                                                                                   helper.getRobotModel().getSimpleRobotName());
      resetInput.addCallback(() ->
                             {
                                isBeingReset.set(true);
                                LogTools.info("Reset requested");

                                bodyPathPlanning.reset();
                                localization.reset();
                                footstepPlanning.reset();
                                stepping.reset();

                                robotInterface.pauseWalking();

                                reset.queueReset();
                             });
      reset.initialize(statusLogger,
                       controllerStatusTracker,
                       lookAndStepParameters,
                       () ->
                       {
                          bodyPathPlanning.acceptGoal(null);
                          behaviorStateReference.set(State.BODY_PATH_PLANNING);
                          lastStanceSide.set(null);
                          helper.publishToUI(ResetForUI);
                          lastCommandedFootsteps.clear();
                          controllerStatusTracker.getFootstepTracker().reset();
                          isBeingReset.set(false);
                          statusLogger.info("Behavior has been reset");
                       });

      // TODO: Implement neck tracker. Make sure neck is down on body path planning entrance

      // TODO: Want to be able to wire up behavior here and see all present modules
      // TODO: Add more meaning to the construction by establishing data patterns
      // TODO: Use named interfaces or pass in actual objects to prevent misordered arguments

      // could add meaning by local variables before passing
      bodyPathPlanning.initialize(
            statusLogger,
            helper::publishToUI,
            robotInterface::pitchHeadWithRespectToChest,
            visibilityGraphParameters,
            lookAndStepParameters,
            operatorReviewEnabledInput::get,
            robotInterface.newSyncedRobot(),
            behaviorStateReference::get,
            controllerStatusTracker,
            bodyPathPlan ->
            {
               if (!isBeingReset.get())
               {
                  behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
                  localization.acceptBodyPathPlan(bodyPathPlan);
               }
            },
            approvalNotification
      );
      helper.createROS2Callback(ROS2Tools.LIDAR_REA_REGIONS, bodyPathPlanning::acceptMapRegions);
      helper.createROS2Callback(GOAL_INPUT, bodyPathPlanning::acceptGoal);
      localization.initialize(
            statusLogger,
            helper::publishToUI,
            lookAndStepParameters,
            robotInterface.newSyncedRobot(),
            () -> {
               if ((!isBeingReset.get()))
               {
                  bodyPathPlanning.acceptGoal(null);
                  helper.publishROS2(REACHED_GOAL);
                  behaviorStateReference.set(LookAndStepBehavior.State.BODY_PATH_PLANNING);
               }
            },
            lastCommandedFootsteps,
            footstepPlanning::acceptLocalizationResult
      );
      helper.createROS2ControllerCallback(CapturabilityBasedStatus.class, localization::acceptCapturabilityBasedStatus);
      footstepPlanning.initialize(
            statusLogger,
            lookAndStepParameters,
            footstepPlannerParameters,
            swingPlannerParameters,
            helper::publishToUI,
            helper.getOrCreateFootstepPlanner(),
            lastStanceSide,
            operatorReviewEnabledInput::get,
            robotInterface.newSyncedRobot(),
            behaviorStateReference::get,
            controllerStatusTracker,
            footstepPlan ->
            {
               if (!isBeingReset.get())
               {
                  behaviorStateReference.set(LookAndStepBehavior.State.STEPPING);
                  stepping.acceptFootstepPlan(footstepPlan);
               }
            },
            approvalNotification
      );
      helper.createROS2Callback(ROS2Tools.REALSENSE_SLAM_REGIONS, footstepPlanning::acceptPlanarRegions);
      helper.createROS2ControllerCallback(FootstepStatusMessage.class, status ->
      {
         if (status.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            footstepPlanning.acceptFootstepCompleted();
         }
      });
      stepping.initialize(
            statusLogger,
            robotInterface.newSyncedRobot(),
            lookAndStepParameters,
            helper::publishToUI,
            robotInterface::requestWalk,
            () ->
            {
               if (!isBeingReset.get())
               {
                  behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
                  localization.acceptSwingSleepComplete();
               }
            },
            controllerStatusTracker,
            behaviorStateReference::get,
            lastCommandedFootsteps
      );
   }

   private void updateOverrideParameters()
   {
      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.getIdealFootstepLengthOverride());
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.getWiggleInsideDeltaOverride());
      footstepPlannerParameters.setCliffBaseHeightToAvoid(lookAndStepParameters.getCliffBaseHeightToAvoidOverride());
      footstepPlannerParameters.setEnableConcaveHullWiggler(lookAndStepParameters.getEnableConcaveHullWigglerOverride());
      swingPlannerParameters.setMinimumSwingFootClearance(lookAndStepParameters.getMinimumSwingFootClearanceOverride());
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      helper.setCommunicationCallbacksEnabled(enabled);
      statusLogger.info("Look and step behavior selected = {}", enabled);
      behaviorStateReference.broadcast();
   }
}
