package us.ihmc.humanoidBehaviors.lookAndStep;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.State.BODY_PATH_PLANNING;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.PlanarRegionsForUI;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.string.StringTools;

public class LookAndStepBodyPathPlanningTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected Supplier<Boolean> operatorReviewEnabled;

   protected final LookAndStepReview<List<? extends Pose3DReadOnly>> review = new LookAndStepReview<>();
   protected Consumer<List<? extends Pose3DReadOnly>> output;

   protected final Timer planningFailedTimer = new Timer();
   protected final Stopwatch planningStopwatch = new Stopwatch();

   public static class LookAndStepBodyPathPlanning extends LookAndStepBodyPathPlanningTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private final TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
      private final TypedInput<Pose3D> goalInput = new TypedInput<>();
      private final Timer mapRegionsExpirationTimer = new Timer();
      private TimerSnapshotWithExpiration mapRegionsReceptionTimerSnapshot;
      private Supplier<LookAndStepBehavior.State> behaviorStateReference;
      private BehaviorTaskSuppressor suppressor;
      private double neckPitch;
      private final Timer neckTrajectoryTimer = new Timer();
      private TimerSnapshotWithExpiration neckTrajectoryTimerSnapshot;
      private TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
      private TimerSnapshotWithExpiration planningFailureTimerSnapshot;
      private LookAndStepBehavior.State behaviorState;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         statusLogger = lookAndStep.statusLogger;
         uiPublisher = lookAndStep.helper::publishToUI;
         visibilityGraphParameters = lookAndStep.visibilityGraphParameters;
         lookAndStepParameters = lookAndStep.lookAndStepParameters;
         operatorReviewEnabled = lookAndStep.operatorReviewEnabledInput::get;
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         behaviorStateReference = lookAndStep.behaviorStateReference::get;
         output = lookAndStep::bodyPathPlanInput;
         ControllerStatusTracker controllerStatusTracker = lookAndStep.controllerStatusTracker;

         Consumer<Double> commandPitchHeadWithRespectToChest = lookAndStep.robotInterface::pitchHeadWithRespectToChest;
         RobotTarget robotTarget = lookAndStep.helper.getRobotModel().getTarget();

         review.initialize(statusLogger, "body path", lookAndStep.approvalNotification, output);

         // don't run two body path plans at the same time
         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         mapRegionsInput.addCallback(data -> executor.submitTask(this::evaluateAndRun));
         goalInput.addCallback(data -> executor.submitTask(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Body path planning");
         suppressor.addCondition("Not in body path planning state", () -> !behaviorState.equals(BODY_PATH_PLANNING));
         if (robotTarget == RobotTarget.SCS)
         {
            statusLogger.info("Robot target is {}. Adding neck suppressor conditions.", robotTarget);
            suppressor.addCondition(() -> "Looking... Neck pitch: " + neckPitch,
                                    () -> neckTrajectoryTimerSnapshot.isRunning());
            suppressor.addCondition(SuppressionConditions.neckPitchWithCorrection(() -> neckPitch,
                                                                                  lookAndStepParameters::getNeckPitchForBodyPath,
                                                                                  lookAndStepParameters::getNeckPitchTolerance,
                                                                                  () ->
                                              {
                                                 commandPitchHeadWithRespectToChest.accept(lookAndStepParameters.getNeckPitchForBodyPath());
                                                 neckTrajectoryTimer.reset();
                                              }));
         }
         suppressor.addCondition("No goal specified",
                                 () -> !(goal != null && !goal.containsNaN()),
                                 () -> uiPublisher.publishToUI(PlanarRegionsForUI, mapRegions));
         suppressor.addCondition(() -> "Regions expired. haveReceivedAny: " + mapRegionsReceptionTimerSnapshot.hasBeenSet()
                                       + " timeSinceLastUpdate: " + mapRegionsReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> mapRegionsReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No regions. "
                                       + (mapRegions == null ? null : (" isEmpty: " + mapRegions.isEmpty())),
                                 () -> !(mapRegions != null && !mapRegions.isEmpty()));
         // TODO: This could be "run recently" instead of failed recently
         suppressor.addCondition("Failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition("Is being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> robotDataReceptionTimerSnaphot.isExpired());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
         suppressor.addCondition("Robot still walking", controllerStatusTracker::isWalking);
      }

      public void acceptMapRegions(PlanarRegionsListMessage planarRegionsListMessage)
      {
         mapRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
         mapRegionsExpirationTimer.reset();
      }

      public void acceptGoal(Pose3D goal)
      {
         goalInput.set(goal);
         LogTools.info(StringTools.format("Body path goal received: {}",
                                          goal == null ? null : StringTools.format("x: {} y: {} z: {} yaw: {}",
                                                                                   goal.getX(),
                                                                                   goal.getY(),
                                                                                   goal.getZ(),
                                                                                   goal.getYaw())
                                                                           .get()));
      }

      public void reset()
      {
         executor.interruptAndReset();
         review.reset();
         goalInput.set(null);
      }

      private void evaluateAndRun()
      {
         mapRegions = mapRegionsInput.getLatest();
         goal = goalInput.getLatest();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepParameters.getRobotConfigurationDataExpiration());
         mapRegionsReceptionTimerSnapshot = mapRegionsExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         planningFailureTimerSnapshot = planningFailedTimer.createSnapshot(lookAndStepParameters.getWaitTimeAfterPlanFailed());
         behaviorState = behaviorStateReference.get();
         neckTrajectoryTimerSnapshot = neckTrajectoryTimer.createSnapshot(1.0);

         neckPitch = syncedRobot.getFramePoseReadOnly(frames -> frames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH)).getOrientation().getPitch();

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }
   }

   protected PlanarRegionsList mapRegions;
   protected Pose3D goal;
   protected RemoteSyncedRobotModel syncedRobot;

   protected void performTask()
   {
      statusLogger.info("Body path planning...");
      // TODO: Add robot standing still for 20s for real robot?
      uiPublisher.publishToUI(PlanarRegionsForUI, mapRegions);

      // calculate and send body path plan
      final ArrayList<Pose3D> bodyPathPlanForReview = new ArrayList<>(); // TODO Review making this final
      Pair<BodyPathPlanningResult, List<Pose3DReadOnly>> result
            = lookAndStepParameters.getFlatGroundBodyPathPlan() ? performTaskWithFlatGround() : performTaskWithVisibilityGraphPlanner();

      statusLogger.info("Body path plan completed with {}, {} waypoint(s)", result.getLeft(), result.getRight().size());

      if (result.getRight() != null)
      {
         for (Pose3DReadOnly poseWaypoint : result.getRight())
         {
            bodyPathPlanForReview.add(new Pose3D(poseWaypoint));
         }
         uiPublisher.publishToUI(BodyPathPlanForUI, bodyPathPlanForReview);
      }

      if (bodyPathPlanForReview.size() >= 2)
      {
         if (operatorReviewEnabled.get())
         {
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

   private Pair<BodyPathPlanningResult, List<Pose3DReadOnly>> performTaskWithVisibilityGraphPlanner()
   {
      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters, pathPostProcessor);

      bodyPathPlanner.setGoal(goal);
      bodyPathPlanner.setPlanarRegionsList(mapRegions);
      bodyPathPlanner.setStanceFootPoses(syncedRobot.getReferenceFrames());
      planningStopwatch.start();
      BodyPathPlanningResult result = bodyPathPlanner.planWaypoints();
      return new MutablePair<>(result, bodyPathPlanner.getWaypoints());// takes about 0.1s
   }

   private Pair<BodyPathPlanningResult, List<Pose3DReadOnly>> performTaskWithFlatGround()
   {
      // calculate and send body path plan
      FramePose3D leftFootPoseTemp = new FramePose3D();
      leftFootPoseTemp.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPoseTemp = new FramePose3D();
      rightFootPoseTemp.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      Pose3D start = new Pose3D();
      start.interpolate(leftFootPoseTemp, rightFootPoseTemp, 0.5);

      List<Pose3DReadOnly> waypoints = new ArrayList<>();
      waypoints.add(start);
      waypoints.add(goal);

      return new MutablePair<>(BodyPathPlanningResult.FOUND_SOLUTION, waypoints);
   }

}
