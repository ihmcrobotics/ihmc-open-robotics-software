package us.ihmc.humanoidBehaviors.lookAndStep;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.State.BODY_PATH_PLANNING;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.PlanarRegionsForUI;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.Timer;
import us.ihmc.communication.util.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
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
import us.ihmc.yoVariables.registry.YoRegistry;

public class LookAndStepBodyPathPlanningTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected Supplier<Boolean> operatorReviewEnabled;

   protected final LookAndStepReview<List<? extends Pose3DReadOnly>> review = new LookAndStepReview<>();
   protected Consumer<List<? extends Pose3DReadOnly>> output;

   protected final Timer planningFailedTimer = new Timer();
   protected final Stopwatch planningStopwatch = new Stopwatch();

   public static class LookAndStepBodyPathPlanning extends LookAndStepBodyPathPlanningTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private ControllerStatusTracker controllerStatusTracker;
      private TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
      private TypedInput<Pose3D> goalInput = new TypedInput<>();
      private Timer mapRegionsExpirationTimer = new Timer();
      private TimerSnapshotWithExpiration mapRegionsReceptionTimerSnapshot;
      private Supplier<LookAndStepBehavior.State> behaviorStateReference;
      private BehaviorTaskSuppressor suppressor;
      private double neckPitch;
      private Timer neckTrajectoryTimer = new Timer();
      private TimerSnapshotWithExpiration neckTrajectoryTimerSnapshot;
      private TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
      private TimerSnapshotWithExpiration planningFailureTimerSnapshot;
      private LookAndStepBehavior.State behaviorState;
      private int numberOfIncompleteFootsteps;

      public void initialize(StatusLogger statusLogger,
                             UIPublisher uiPublisher,
                             Consumer<Double> commandPitchHeadWithRespectToChest,
                             VisibilityGraphsParametersReadOnly visibilityGraphParameters,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             Supplier<Boolean> operatorReviewEnabled,
                             RemoteSyncedRobotModel syncedRobot,
                             Supplier<LookAndStepBehavior.State> behaviorStateReference,
                             ControllerStatusTracker controllerStatusTracker,
                             Consumer<List<? extends Pose3DReadOnly>> output,
                             TypedNotification<Boolean> approvalNotification)
      {
         this.statusLogger = statusLogger;
         this.uiPublisher = uiPublisher;
         this.visibilityGraphParameters = visibilityGraphParameters;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.operatorReviewEnabled = operatorReviewEnabled;
         this.syncedRobot = syncedRobot;
         this.behaviorStateReference = behaviorStateReference;
         this.output = output;
         this.controllerStatusTracker = controllerStatusTracker;

         review.initialize(statusLogger, "body path", approvalNotification, output);

         // don't run two body path plans at the same time
         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         mapRegionsInput.addCallback(data -> executor.queueExecution(this::evaluateAndRun));
         goalInput.addCallback(data -> executor.queueExecution(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Body path planning");
         suppressor.addCondition("Not in body path planning state", () -> !behaviorState.equals(BODY_PATH_PLANNING));
         suppressor.addCondition(() -> "Looking... Neck pitch: " + neckPitch,
                                 () -> neckTrajectoryTimerSnapshot.isRunning());
         suppressor.addCondition(() -> "Neck at wrong angle: " + neckPitch
                                       + " != " + lookAndStepBehaviorParameters.getNeckPitchForBodyPath()
                                       + " +/- " + lookAndStepBehaviorParameters.getNeckPitchTolerance(),
                                 () -> Math.abs(neckPitch - lookAndStepBehaviorParameters.getNeckPitchForBodyPath())
                                       > lookAndStepBehaviorParameters.getNeckPitchTolerance(),
                                 () ->
                                 {
                                    commandPitchHeadWithRespectToChest.accept(lookAndStepBehaviorParameters.getNeckPitchForBodyPath());
                                    neckTrajectoryTimer.reset();
                                 });
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
         LogTools.info("Body path goal received: {}", goal);
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
                                                     .withExpiration(lookAndStepBehaviorParameters.getRobotConfigurationDataExpiration());
         mapRegionsReceptionTimerSnapshot = mapRegionsExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.getPlanarRegionsExpiration());
         planningFailureTimerSnapshot = planningFailedTimer.createSnapshot(lookAndStepBehaviorParameters.getWaitTimeAfterPlanFailed());
         behaviorState = behaviorStateReference.get();
         numberOfIncompleteFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps();
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
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      YoRegistry parentRegistry = new YoRegistry(getClass().getSimpleName());
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters, pathPostProcessor, parentRegistry);

      bodyPathPlanner.setGoal(goal);
      bodyPathPlanner.setPlanarRegionsList(mapRegions);
      FramePose3D leftFootPoseTemp = new FramePose3D();
      leftFootPoseTemp.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPoseTemp = new FramePose3D();
      rightFootPoseTemp.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPathPlanner.setStanceFootPoses(leftFootPoseTemp, rightFootPoseTemp);
      final ArrayList<Pose3D> bodyPathPlanForReview = new ArrayList<>(); // TODO Review making this final
      planningStopwatch.start();
      BodyPathPlanningResult result = bodyPathPlanner.planWaypoints();// takes about 0.1s
      statusLogger.info("Body path plan completed with {}, {} waypoint(s)", result, bodyPathPlanner.getWaypoints().size());
      //      bodyPathPlan = bodyPathPlanner.getWaypoints();
      if (bodyPathPlanner.getWaypoints() != null)
      {
         for (Pose3DReadOnly poseWaypoint : bodyPathPlanner.getWaypoints())
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
}
