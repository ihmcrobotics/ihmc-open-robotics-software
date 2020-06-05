package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.lookAndStep.parts.LookAndStepBodyPathModule;
import us.ihmc.humanoidBehaviors.lookAndStep.parts.LookAndStepFootstepPlanningModule;
import us.ihmc.humanoidBehaviors.lookAndStep.parts.LookAndStepReviewPart;
import us.ihmc.humanoidBehaviors.lookAndStep.parts.LookAndStepRobotMotionModule;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequest;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   private final BehaviorHelper helper;
   private final RemoteHumanoidRobotInterface robot;

   private final LookAndStepBodyPathModule bodyPathModule;
   private final LookAndStepFootstepPlanningModule footstepPlanningModule;
   private final LookAndStepRobotMotionModule robotMotionModule;

   public enum State
   {
      BODY_PATH_PLANNING, FOOTSTEP_PLANNING, SWINGING
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

      robot = helper.getOrCreateRobotInterface();

      VisibilityGraphsParametersBasics visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();
      visibilityGraphParameters.setIncludePreferredExtrusions(false);

      LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      FootstepPlannerParametersBasics footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      // hook up override parameters
      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.getIdealFootstepLengthOverride());
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.getWiggleInsideDeltaOverride());
      footstepPlannerParameters.setCliffBaseHeightToAvoid(lookAndStepParameters.getCliffBaseHeightToAvoidOverride());
      footstepPlannerParameters.setEnableConcaveHullWiggler(lookAndStepParameters.getEnableConcaveHullWigglerOverride());

      AtomicReference<Boolean> operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      TypedNotification<Boolean> approvalNotification = helper.createUITypedNotification(Approval);

      AtomicBoolean newBodyPathGoalNeeded = new AtomicBoolean(true);
      AtomicReference<RobotSide> lastStanceSide = new AtomicReference<>();
      SideDependentList<FramePose3DReadOnly> lastSteppedSolePoses = new SideDependentList<>();

      AtomicReference<State> behaviorState = new AtomicReference<>(State.BODY_PATH_PLANNING);

      // TODO: Want to be able to wire up behavior here and see all present modules

      bodyPathModule = new LookAndStepBodyPathModule();
      footstepPlanningModule = new LookAndStepFootstepPlanningModule();
      robotMotionModule = new LookAndStepRobotMotionModule();

      LookAndStepReviewPart<List<? extends Pose3DReadOnly>> bodyPathReview = new LookAndStepReviewPart<>("body path", approvalNotification, bodyPathPlan ->
      {
         behaviorState.set(State.FOOTSTEP_PLANNING);
         footstepPlanningModule.acceptBodyPathPlan(bodyPathPlan);
      });
      LookAndStepReviewPart<RobotWalkRequest> footstepPlanReview = new LookAndStepReviewPart<>("footstep plan", approvalNotification, robotWalkRequest ->
      {
         behaviorState.set(LookAndStepBehavior.State.SWINGING);
         robotMotionModule.acceptRobotWalkRequest(robotWalkRequest);
      });

      bodyPathModule.setRobotStateSupplier(robot::pollHumanoidRobotState);
      bodyPathModule.setIsBeingReviewedSupplier(bodyPathReview::isBeingReviewed);
      bodyPathModule.setOperatorReviewEnabled(operatorReviewEnabledInput::get);
      bodyPathModule.setLookAndStepBehaviorParameters(lookAndStepParameters);
      bodyPathModule.setVisibilityGraphParameters(visibilityGraphParameters);
      bodyPathModule.setReviewInitiator(bodyPathReview::review);
      bodyPathModule.setAutonomousOutput(footstepPlanningModule::acceptBodyPathPlan);
      bodyPathModule.setNeedNewPlan(newBodyPathGoalNeeded::get); // TODO: hook up to subgoal mover
      bodyPathModule.setClearNewBodyPathGoalNeededCallback(() -> newBodyPathGoalNeeded.set(false));
      bodyPathModule.setUIPublisher(helper::publishToUI);
      bodyPathModule.setBehaviorStateSupplier(behaviorState::get);
      bodyPathModule.setBehaviorStateUpdater(behaviorState::set);

      footstepPlanningModule.setIsBeingReviewedSupplier(footstepPlanReview::isBeingReviewed);
      footstepPlanningModule.setUiPublisher(helper::publishToUI);
      footstepPlanningModule.setLookAndStepBehaviorParameters(lookAndStepParameters);
      footstepPlanningModule.setFootstepPlannerParameters(footstepPlannerParameters);
      footstepPlanningModule.setNewBodyPathGoalNeededNotifier(() -> bodyPathModule.acceptGoal(null));
      footstepPlanningModule.setNewBodyPathGoalNeededSupplier(newBodyPathGoalNeeded::get);
      footstepPlanningModule.setLastStanceSideSupplier(lastStanceSide::get);
      footstepPlanningModule.setLastStanceSideSetter(lastStanceSide::set);
      footstepPlanningModule.setLastSteppedSolePoseSupplier(lastSteppedSolePoses::get);
      footstepPlanningModule.setLastSteppedSolePoseConsumer(lastSteppedSolePoses::put);
      footstepPlanningModule.setOperatorReviewEnabledSupplier(operatorReviewEnabledInput::get);
      footstepPlanningModule.setReviewPlanOutput(footstepPlanReview::review);
      footstepPlanningModule.setAutonomousOutput(robotMotionModule::acceptRobotWalkRequest);
      footstepPlanningModule.setRobotStateSupplier(robot::pollHumanoidRobotState);
      footstepPlanningModule.setFootstepPlanningModule(helper.getOrCreateFootstepPlanner());
      footstepPlanningModule.setBehaviorStateSupplier(behaviorState::get);
      footstepPlanningModule.setBehaviorStateUpdater(behaviorState::set);

      robotMotionModule.setRobotStateSupplier(robot::pollHumanoidRobotState);
      robotMotionModule.setLastSteppedSolePoseConsumer(lastSteppedSolePoses::put);
      robotMotionModule.setLastSteppedSolePoseSupplier(lastSteppedSolePoses::get);
      robotMotionModule.setLookAndStepBehaviorParameters(lookAndStepParameters);
      robotMotionModule.setReplanFootstepsOutput(footstepPlanningModule::evaluateAndRun);
      robotMotionModule.setRobotWalkRequester(robot::requestWalk);
      robotMotionModule.setUiPublisher(helper::publishToUI);
      robotMotionModule.setBehaviorStateSupplier(behaviorState::get);
      robotMotionModule.setBehaviorStateUpdater(behaviorState::set);

      // TODO: For now, these cause trouble if they are setup earlier. Need to disable them on creation
      helper.createROS2Callback(ROS2Tools.MAP_REGIONS, bodyPathModule::acceptMapRegions);
      helper.createUICallback(GoalInput, bodyPathModule::acceptGoal);
      helper.createROS2Callback(ROS2Tools.REALSENSE_SLAM_REGIONS, footstepPlanningModule::acceptPlanarRegions);

      helper.setCommunicationCallbacksEnabled(false);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Look and step behavior selected = {}", enabled);

      helper.setCommunicationCallbacksEnabled(enabled);

      robot.pitchHeadWithRespectToChest(0.38);
      //      Commanding neck trajectory: slider: 43.58974358974359 angle: 0.3824055641025641
   }
}
