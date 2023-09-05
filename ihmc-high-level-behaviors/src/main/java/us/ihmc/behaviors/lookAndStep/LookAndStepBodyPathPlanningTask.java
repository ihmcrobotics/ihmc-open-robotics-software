package us.ihmc.behaviors.lookAndStep;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehavior.State.BODY_PATH_PLANNING;
import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import ihmc_common_msgs.msg.dds.PoseListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.interfaces.UIPublisher;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class LookAndStepBodyPathPlanningTask
{
   protected BehaviorHelper helper;
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected Supplier<Boolean> operatorReviewEnabled;
   protected FootstepPlanningModule footstepPlanningModule;
   protected final FramePose3D startFramePose = new FramePose3D();
   protected final FramePose3D goalFramePose = new FramePose3D();

   protected final LookAndStepReview<List<? extends Pose3DReadOnly>> review = new LookAndStepReview<>();
   protected Consumer<List<? extends Pose3DReadOnly>> output;

   protected final Timer planningFailedTimer = new Timer();
   protected double bodyPathPlanningDuration;

   protected PlanarRegionsList mapRegions;
   protected HeightMapData heightMap;
   protected Pose3DReadOnly goal;
   protected ROS2SyncedRobotModel syncedRobot;
   protected boolean doBodyPathPlan;
   protected RigidBodyTransform goalToWorld = new RigidBodyTransform();
   protected ReferenceFrame goalFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), goalToWorld);

   public static class LookAndStepBodyPathPlanning extends LookAndStepBodyPathPlanningTask
   {
      private ResettableExceptionHandlingExecutorService executor;
      private final TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
      private final TypedInput<HeightMapData> heightMapInput = new TypedInput<>();
      private final TypedInput<Pose3DReadOnly> goalInput = new TypedInput<>();
      private final Timer mapRegionsExpirationTimer = new Timer();
      private final Timer heightMapExpirationTimer = new Timer();
      private TimerSnapshotWithExpiration mapRegionsReceptionTimerSnapshot;
      private Supplier<LookAndStepBehavior.State> behaviorStateReference;
      private BehaviorTaskSuppressor suppressor;
      private double neckPitch;
      private final Timer neckTrajectoryTimer = new Timer();
      private TimerSnapshotWithExpiration neckTrajectoryTimerSnapshot;
      private TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
      private TimerSnapshotWithExpiration planningFailureTimerSnapshot;
      private LookAndStepBehavior.State behaviorState;
      private ROS2StoredPropertySet<LookAndStepBehaviorParametersBasics> ros2LookAndStepParameters;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         statusLogger = lookAndStep.statusLogger;
         helper = lookAndStep.helper;
         uiPublisher = lookAndStep.helper::publish;
         visibilityGraphParameters = lookAndStep.visibilityGraphParameters;
         ros2LookAndStepParameters = lookAndStep.ros2LookAndStepParameters;
         lookAndStepParameters = ros2LookAndStepParameters.getStoredPropertySet();
         operatorReviewEnabled = lookAndStep.operatorReviewEnabledInput::get;
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         behaviorStateReference = lookAndStep.behaviorStateReference::get;
         output = lookAndStep::bodyPathPlanInput;
         ControllerStatusTracker controllerStatusTracker = lookAndStep.controllerStatusTracker;
         footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());

         Consumer<Double> commandPitchHeadWithRespectToChest = lookAndStep.robotInterface::pitchHeadWithRespectToChest;
         RobotTarget robotTarget = lookAndStep.helper.getRobotModel().getTarget();

         review.initialize(statusLogger, "body path", lookAndStep.approvalNotification, output);

         // don't run two body path plans at the same time
         executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

         mapRegionsInput.addCallback(data -> run());
         heightMapInput.addCallback(data -> run());
         goalInput.addCallback(data -> run());

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Body path planning");
         suppressor.addCondition("Not in body path planning state", () -> !behaviorState.equals(BODY_PATH_PLANNING));
//         if (robotTarget == RobotTarget.SCS)
//         {
//            statusLogger.info("Robot target is {}. Adding neck suppressor conditions.", robotTarget);
//            suppressor.addCondition(() -> "Looking... Neck pitch: " + neckPitch,
//                                    () -> neckTrajectoryTimerSnapshot.isRunning());
//            suppressor.addCondition(SuppressionConditions.neckPitchWithCorrection(() -> neckPitch,
//                                                                                  lookAndStepParameters::getNeckPitchForBodyPath,
//                                                                                  lookAndStepParameters::getNeckPitchTolerance,
//                                                                                  () ->
//                                              {
//                                                 commandPitchHeadWithRespectToChest.accept(lookAndStepParameters.getNeckPitchForBodyPath());
//                                                 neckTrajectoryTimer.reset();
//                                              }));
//         }
         suppressor.addCondition("No goal specified",
                                 () -> !(goal != null && !goal.containsNaN()),
                                 () ->
                                 {
                                    if (mapRegions != null)
                                       helper.publish(PLANAR_REGIONS_FOR_UI, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(mapRegions));
                                 });
//         suppressor.addCondition(() -> "Regions expired. haveReceivedAny: " + mapRegionsReceptionTimerSnapshot.hasBeenSet()
//                                       + " timeSinceLastUpdate: " + mapRegionsReceptionTimerSnapshot.getTimePassedSinceReset(),
//                                 () -> mapRegionsReceptionTimerSnapshot.isExpired());
//         suppressor.addCondition(() -> "No regions. "
//                                       + (mapRegions == null ? null : (" isEmpty: " + mapRegions.isEmpty())),
//                                 () -> !(mapRegions != null && !mapRegions.isEmpty()));
         // TODO: This could be "run recently" instead of failed recently
         suppressor.addCondition("Failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition("Is being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> robotDataReceptionTimerSnaphot.isExpired());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
//         suppressor.addCondition("Robot still walking", controllerStatusTracker::isWalking);
      }

      public void acceptMapRegions(PlanarRegionsListMessage planarRegionsListMessage)
      {
         mapRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
         mapRegionsExpirationTimer.reset();
      }

      public void acceptHeightMap(HeightMapMessage heightMapMessage)
      {
         heightMapInput.set(HeightMapMessageTools.unpackMessage(heightMapMessage));
         heightMapExpirationTimer.reset();
      }

      public void acceptGoal(Pose3DReadOnly goal)
      {
         reset();
         goalInput.set(goal);
         LogTools.info(StringTools.format("Body path goal received: {}",
                                          goal == null ? null : StringTools.format("x: {} y: {} z: {} yaw: {}",
                                                                                   goal.getX(),
                                                                                   goal.getY(),
                                                                                   goal.getZ(),
                                                                                   goal.getYaw())
                                                                           .get()));
      }

      public void run()
      {
         executor.clearQueueAndExecute(this::evaluateAndRun);
      }

      public boolean isReset()
      {
         return goalInput.getLatest() == null;
      }

      public void reset()
      {
         executor.interruptAndReset();
         review.reset();
         goalInput.set(null);
      }

      private void evaluateAndRun()
      {
         ros2LookAndStepParameters.update();
         mapRegions = mapRegionsInput.getLatest();
         heightMap = heightMapInput.getLatest();
         goal = goalInput.getLatest();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepParameters.getRobotConfigurationDataExpiration());
         mapRegionsReceptionTimerSnapshot = mapRegionsExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         planningFailureTimerSnapshot = planningFailedTimer.createSnapshot(lookAndStepParameters.getWaitTimeAfterPlanFailed());
         behaviorState = behaviorStateReference.get();
         neckTrajectoryTimerSnapshot = neckTrajectoryTimer.createSnapshot(1.0);
         doBodyPathPlan = !lookAndStepParameters.getFlatGroundBodyPathPlan();

         // neckPitch = syncedRobot.getFramePoseReadOnly(frames -> frames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH)).getOrientation().getPitch();

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }

      public void destroy()
      {
         executor.destroy();
      }
   }

   protected final Stopwatch planningStopwatch = new Stopwatch();

   protected void performTask()
   {
      statusLogger.info("Body path planning...");
      planningStopwatch.reset();
      final ArrayList<Pose3D> bodyPathPlanForReview = new ArrayList<>(); // TODO Review making this final
      Pair<BodyPathPlanningResult, List<? extends Pose3DReadOnly>> result;
      if (doBodyPathPlan && (mapRegions != null || heightMap != null))
      {
         if (mapRegions != null)
         {
            helper.publish(PLANAR_REGIONS_FOR_UI, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(mapRegions));
            result = performTaskWithVisibilityGraphPlanner();
         }
         else
         {
            result = performTaskWithHeightMapPlanner();
         }
      }
      else // flat ground body path
      {
         result = performTaskWithFlatGround();
      }

      bodyPathPlanningDuration = planningStopwatch.lap();
      double pathLength = BodyPathPlannerTools.calculatePlanLength(result.getRight());
      statusLogger.info(StringTools.format("Body path plan completed with {}, {} waypoint(s), length: {}, planning duration: {} s",
                                           result.getLeft(),
                                           result.getRight().size(),
                                           FormattingTools.getFormattedDecimal2D(pathLength),
                                           bodyPathPlanningDuration));

      if (result.getRight() != null)
      {
         PoseListMessage bodyPathPlanForReviewMessage = new PoseListMessage();
         for (Pose3DReadOnly poseWaypoint : result.getRight())
         {
            bodyPathPlanForReview.add(new Pose3D(poseWaypoint));
            bodyPathPlanForReviewMessage.getPoses().add().set(poseWaypoint);
         }
         helper.publish(BODY_PATH_PLAN_FOR_UI, bodyPathPlanForReviewMessage);
      }

      if (bodyPathPlanForReview.size() >= 2)
      {
         if (operatorReviewEnabled.get())
         {
            if (lookAndStepParameters.getMaxStepsToSendToController() > 1)
               helper.getOrCreateRobotInterface().pauseWalking();
            review.review(bodyPathPlanForReview);
         }
         else
         {
            output.accept(bodyPathPlanForReview);
         }
      }
      else
      {
         planningFailedTimer.reset();
      }
   }

   private Pair<BodyPathPlanningResult, List<? extends Pose3DReadOnly>> performTaskWithVisibilityGraphPlanner()
   {
      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters, pathPostProcessor);

      bodyPathPlanner.setGoal(goal);
      bodyPathPlanner.setPlanarRegionsList(mapRegions);
      bodyPathPlanner.setStanceFootPoses(syncedRobot.getReferenceFrames());
      BodyPathPlanningResult result = bodyPathPlanner.planWaypoints();
      return new MutablePair<>(result, bodyPathPlanner.getWaypoints());// takes about 0.1s
   }

//   private Pair<BodyPathPlanningResult, List<Pose3DReadOnly>> performTaskWithHeightMapPlanner()
//   {
//
//   }

   private Pair<BodyPathPlanningResult, List<? extends Pose3DReadOnly>> performTaskWithFlatGround()
   {
      double proximityForTurning = 0.25;

      // calculate and send body path plan
      FramePose3D leftFootPoseTemp = new FramePose3D();
      leftFootPoseTemp.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPoseTemp = new FramePose3D();
      rightFootPoseTemp.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      Pose3D start = new Pose3D();
      start.interpolate(leftFootPoseTemp, rightFootPoseTemp, 0.5);

      Vector3D goalDirection = new Vector3D();
      goalDirection.sub(goal.getPosition(), start.getPosition());
      double goalDistance = goalDirection.length();

      goalDirection.scale(proximityForTurning / goalDirection.length());
      double heading = BodyPathPlannerTools.calculateHeading(new Vector2D(goalDirection));


      Pose3D poseNearStart = new Pose3D();
      poseNearStart.getPosition().set(start.getPosition());
      poseNearStart.getPosition().add(goalDirection);
      poseNearStart.getOrientation().setToYawOrientation(heading);

      Pose3D poseNearGoal = new Pose3D();
      poseNearGoal.getPosition().set(goal.getPosition());
      poseNearGoal.getPosition().sub(goalDirection);
      poseNearGoal.getOrientation().setToYawOrientation(heading);

      List<Pose3DReadOnly> waypoints = new ArrayList<>();
      waypoints.add(start);
      if (goalDistance > 2.0 * proximityForTurning)
      {
         waypoints.add(poseNearStart);
         waypoints.add(poseNearGoal);
      }
      waypoints.add(goal);

      return new MutablePair<>(BodyPathPlanningResult.FOUND_SOLUTION, waypoints);
   }

   private Pair<BodyPathPlanningResult, List<? extends Pose3DReadOnly>> performTaskWithHeightMapPlanner()
   {
      goalToWorld.set(goal);
      goalFrame.update();

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanFootsteps(false);

      startFramePose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      startFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      footstepPlannerRequest.getStartFootPoses().put(RobotSide.LEFT, new Pose3D(startFramePose));
      startFramePose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      startFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      footstepPlannerRequest.getStartFootPoses().put(RobotSide.RIGHT, new Pose3D(startFramePose));
      goalFramePose.setToZero(goalFrame);
      goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      LogTools.info(StringTools.tupleString(goalFramePose.getPosition()));
      LogTools.info("Yaw: {}", goalFramePose.getOrientation().getYaw());
      goalFramePose.setToZero(goalFrame);
      goalFramePose.getPosition().setY(0.2);
      goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      footstepPlannerRequest.getGoalFootPoses().put(RobotSide.LEFT, new Pose3D(goalFramePose));
      goalFramePose.setToZero(goalFrame);
      goalFramePose.getPosition().setY(-0.2);
      goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      footstepPlannerRequest.getGoalFootPoses().put(RobotSide.RIGHT, new Pose3D(goalFramePose));
      footstepPlannerRequest.setTimeout(10.0);

//      heightMapMessage.setEstimatedGroundHeight(-1.0);
      footstepPlannerRequest.setHeightMapData(heightMap);
      footstepPlannerRequest.setPlanBodyPath(true);
      footstepPlannerRequest.setPlanFootsteps(false);
      footstepPlanningModule.handleRequest(footstepPlannerRequest);
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();

      return new MutablePair<>(footstepPlanningModule.getOutput().getBodyPathPlanningResult(), footstepPlanningModule.getOutput().getBodyPath());
   }
}
