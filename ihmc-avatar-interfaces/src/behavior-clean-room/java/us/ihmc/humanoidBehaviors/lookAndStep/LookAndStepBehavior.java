package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
//   private final FootstepPlanningModule footstepPlanningModule;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;
   private final Supplier<PlanarRegionsList> realsenseSLAMRegions;

   private final LookAndStepReviewPart<List<? extends Pose3DReadOnly>> bodyPathReview;
   private final LookAndStepReviewPart<FootstepPlan> footstepPlanReview;

   private final LookAndStepBodyPathModule bodyPathModule;
   private final LookAndStepFootstepPlanningModule footstepPlanningModule;

   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private final TypedNotification<Boolean> approvalNotification;
   private final FramePose3D goalPoseBetweenFeet = new FramePose3D();
   private AtomicReference<RobotSide> lastStanceSide = new AtomicReference<>();
   private volatile SideDependentList<FramePose3DReadOnly> lastSteppedSolePoses = new SideDependentList<>();

   private PlanarRegionsList footstepPlanningNearRegions;
   private SimpleTimer footstepPlanningNearRegionsExpirationTimer = new SimpleTimer();
   private volatile List<Pose3D> footstepPlanningBodyPathPlan;
   private SimpleTimer footstepPlanningModuleFailedTimer = new SimpleTimer();

   private FootstepPlan robotWalkingModuleFootstepPlan;

   /**
    * We'll make the following assumptions/constraints about the data being passed around between task/modules:
    * - Data output from module will be read only and the underlying data must not be modified after sending
    * - Timers are considered parallel tasks/ modules
    *
    * Inputs can be polled or triggered by an event
    * Outputs can be an event or polled by another task
    */
   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      robot = helper.getOrCreateRobotInterface();
//      footstepPlanningModule = helper.getOrCreateFootstepPlanner();
      realsenseSLAMRegions = helper.createROS2PlanarRegionsListInput(ROS2Tools.REALSENSE_SLAM_REGIONS);

      visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();
      visibilityGraphParameters.setIncludePreferredExtrusions(false);

      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      // hook up override parameters
      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.getIdealFootstepLengthOverride());
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.getWiggleInsideDeltaOverride());
      footstepPlannerParameters.setCliffBaseHeightToAvoid(lookAndStepParameters.getCliffBaseHeightToAvoidOverride());
      footstepPlannerParameters.setEnableConcaveHullWiggler(lookAndStepParameters.getEnableConcaveHullWigglerOverride());

      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      approvalNotification = helper.createUITypedNotification(Approval);

      AtomicBoolean newBodyPathGoalNeeded = new AtomicBoolean(true);

      // TODO: Want to be able to wire up behavior here and see all present modules

      bodyPathModule = new LookAndStepBodyPathModule();
      footstepPlanningModule = new LookAndStepFootstepPlanningModule();

      bodyPathReview = new LookAndStepReviewPart<>("body path", approvalNotification, footstepPlanningModule::acceptBodyPathPlan);
      footstepPlanReview = new LookAndStepReviewPart<>("footstep plan", approvalNotification, robotModule::acceptFootstepPlan);

      helper.createROS2Callback(ROS2Tools.MAP_REGIONS, bodyPathModule::acceptMapRegions);
      helper.createUICallback(GoalInput, bodyPathModule::acceptGoal);
      bodyPathModule.setRobotStateSupplier(robot::pollHumanoidRobotState);
      bodyPathModule.setIsBeingReviewedSupplier(bodyPathReview::isBeingReviewed);
      bodyPathModule.setReviewEnabledSupplier(operatorReviewEnabledInput::get);
      bodyPathModule.setLookAndStepBehaviorParameters(lookAndStepParameters);
      bodyPathModule.setVisibilityGraphParameters(visibilityGraphParameters);
      bodyPathModule.setInitiateReviewOutput(bodyPathReview::review);
      bodyPathModule.setAutonomousOutput(footstepPlanningModule::acceptBodyPathPlan);
      bodyPathModule.setNeedNewPlanSupplier(newBodyPathGoalNeeded::get); // TODO: hook up to subgoal mover
      bodyPathModule.setUIPublisher(helper::publishToUI);

      helper.createROS2Callback(ROS2Tools.REALSENSE_SLAM_REGIONS, footstepPlanningModule::acceptPlanarRegions);
      footstepPlanningModule.setIsBeingReviewedSupplier(footstepPlanReview::isBeingReviewed);
      footstepPlanningModule.setUiPublisher(helper::publishToUI);
      footstepPlanningModule.setLookAndStepBehaviorParameters(lookAndStepParameters);
      footstepPlanningModule.setFootstepPlannerParameters(footstepPlannerParameters);
      footstepPlanningModule.setNewBodyPathGoalNeededNotifier(() -> newBodyPathGoalNeeded.set(true));
      footstepPlanningModule.setLastStanceSideSupplier(lastStanceSide::get);
      footstepPlanningModule.setLastStanceSideSetter(lastStanceSide::set);
      footstepPlanningModule.setLastSteppedSolePoseSupplier(lastSteppedSolePoses::get);
      footstepPlanningModule.setLastSteppedSolePoseConsumer(lastSteppedSolePoses::put);
      footstepPlanningModule.setOperatorReviewEnabledSupplier(operatorReviewEnabledInput::get);
      footstepPlanningModule.setReviewPlanOutput(footstepPlanReview::review);
      footstepPlanningModule.setAutonomousOutput(robotModule::acceptFootstepPlan);

      helper.createROS2PlanarRegionsListCallback(ROS2Tools.REALSENSE_SLAM_REGIONS, this::footstepPlanningAcceptNearPlanarRegions);

//      helper.createUICallback(RePlan, this::);

      helper.setCommunicationCallbacksEnabled(true);
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

   private void robotWalkingModuleAcceptFootstepPlan(FootstepPlan footstepPlan)
   {
      LogTools.debug("robotWalkingModuleAcceptFootstepPlan, {} steps", footstepPlan.getNumberOfSteps());
      robotWalkingModuleFootstepPlan = footstepPlan;

      robotWalkingEvaluateAndRun();
   }

   private void robotWalkingEvaluateAndRun()
   {
      boolean footstepPlanOK = robotWalkingModuleFootstepPlan != null && robotWalkingModuleFootstepPlan.getNumberOfSteps() > 0;
      if (!footstepPlanOK)
      {
         LogTools.warn("Robot walking: Footstep plan not OK {}, numberOfSteps = {}. Planning again...",
                       robotWalkingModuleFootstepPlan,
                       robotWalkingModuleFootstepPlan == null ? null : robotWalkingModuleFootstepPlan.getNumberOfSteps());
         footstepPlanningEvaluateAndRun();
         return;
      }

      FootstepPlan shortenedFootstepPlan = new FootstepPlan();
      if (robotWalkingModuleFootstepPlan.getNumberOfSteps() > 0)
      {
         SimpleFootstep footstepToTake = robotWalkingModuleFootstepPlan.getFootstep(0);
         shortenedFootstepPlan.addFootstep(footstepToTake);
         lastSteppedSolePoses.put(footstepToTake.getRobotSide(), new FramePose3D(footstepToTake.getSoleFramePose()));
         sendLastSteppedSolePoses();
      }

      LogTools.info("Requesting walk");
      double swingTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.swingTime);
      double transferTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.transferTime);
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingTime,
                                                                                                                    transferTime,
                                                                                                                    ExecutionMode.QUEUE);
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robot.requestWalk(footstepDataListMessage,
                                                                                            robot.pollHumanoidRobotState(),
                                                                                            realsenseSLAMRegions.get());

      helper.publishToUI(FootstepPlanForUI,
                         reduceFootstepPlanForUIMessager2(FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage), "Stepping"));

      ThreadTools.startAsDaemon(() -> sleepForPartOfSwingThread(swingTime), "RobotWalking");
      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
   }

   private void sleepForPartOfSwingThread(double swingTime)
   {
      double percentSwingToWait = lookAndStepParameters.get(LookAndStepBehaviorParameters.percentSwingToWait);
      double waitTime = swingTime * percentSwingToWait;
      LogTools.info("Waiting {} for {} % of swing...", waitTime, percentSwingToWait);
      ThreadTools.sleepSeconds(waitTime);
      LogTools.info("{} % of swing complete!", percentSwingToWait);

      LogTools.warn("Step {}% complete: Robot not reached goal: Find next footstep planning goal...", percentSwingToWait);
      footstepPlanningEvaluateAndRun();
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      LogTools.info("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      LogTools.info("Robot walk complete.");
   }
}
