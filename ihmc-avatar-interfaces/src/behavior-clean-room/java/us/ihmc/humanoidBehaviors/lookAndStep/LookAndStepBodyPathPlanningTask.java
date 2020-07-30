package us.ihmc.humanoidBehaviors.lookAndStep;

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
import us.ihmc.humanoidBehaviors.tools.walking.WalkingFootstepTracker;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.State.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBodyPathPlanningTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected Supplier<Boolean> operatorReviewEnabled;
   protected BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference;

   protected final LookAndStepReview<List<? extends Pose3DReadOnly>> review = new LookAndStepReview<>();
   protected Consumer<List<? extends Pose3DReadOnly>> autonomousOutput;

   protected final Timer planningFailedTimer = new Timer();
   protected final Stopwatch planningStopwatch = new Stopwatch();

   public static class LookAndStepBodyPathPlanning extends LookAndStepBodyPathPlanningTask
   {
      private WalkingFootstepTracker walkingFootstepTracker;
      private TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
      private TypedInput<Pose3D> goalInput = new TypedInput<>();
      private Timer mapRegionsExpirationTimer = new Timer();
      private TimerSnapshotWithExpiration mapRegionsReceptionTimerSnapshot;
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
                             BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference,
                             WalkingFootstepTracker walkingFootstepTracker,
                             Consumer<List<? extends Pose3DReadOnly>> autonomousOutput,
                             TypedNotification<Boolean> approvalNotification)
      {
         this.statusLogger = statusLogger;
         this.uiPublisher = uiPublisher;
         this.visibilityGraphParameters = visibilityGraphParameters;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.operatorReviewEnabled = operatorReviewEnabled;
         this.syncedRobot = syncedRobot;
         this.behaviorStateReference = behaviorStateReference;
         this.autonomousOutput = autonomousOutput;

         review.initialize(
               statusLogger,
               "body path",
               approvalNotification,
               bodyPathPlan ->
               {
                  behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
                  autonomousOutput.accept(bodyPathPlan);
               }
         );

         this.walkingFootstepTracker = walkingFootstepTracker;

         // don't run two body path plans at the same time
         SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         mapRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));
         goalInput.addCallback(data -> executor.execute(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Body path planning");
         suppressor.addCondition("Not in body path planning state", () -> !behaviorState.equals(BODY_PATH_PLANNING));
         suppressor.addCondition("No goal specified",
                                 () -> !(goal != null && !goal.containsNaN()),
                                 () -> uiPublisher.publishToUI(BodyPathRegionsForUI, mapRegions));
         suppressor.addCondition(() -> "Body path planning suppressed: Regions not OK: " + mapRegions
                                       + ", timePassed: " + mapRegionsReceptionTimerSnapshot.getTimePassedSinceReset()
                                       + ", isEmpty: " + (mapRegions == null ? null : mapRegions.isEmpty()),
                                 () -> !(mapRegions != null && !mapRegions.isEmpty() && mapRegionsReceptionTimerSnapshot.isRunning()));
         suppressor.addCondition(() -> "Looking... Neck pitch: " + neckPitch,
                                 () -> neckTrajectoryTimerSnapshot != null && neckTrajectoryTimerSnapshot.isRunning());
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
         // TODO: This could be "run recently" instead of failed recently
         suppressor.addCondition("Failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition("Is being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> !robotDataReceptionTimerSnaphot.isRunning());
         suppressor.addCondition(() -> "numberOfIncompleteFootsteps " + numberOfIncompleteFootsteps
                                       + " > " + lookAndStepBehaviorParameters.getAcceptableIncompleteFootsteps(),
                                 () -> numberOfIncompleteFootsteps > lookAndStepBehaviorParameters.getAcceptableIncompleteFootsteps());
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

      private void evaluateAndRun()
      {
         mapRegions = mapRegionsInput.get();
         goal = goalInput.get();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepBehaviorParameters.getRobotConfigurationDataExpiration());
         mapRegionsReceptionTimerSnapshot = mapRegionsExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.getPlanarRegionsExpiration());
         planningFailureTimerSnapshot = planningFailedTimer.createSnapshot(lookAndStepBehaviorParameters.getWaitTimeAfterPlanFailed());
         behaviorState = behaviorStateReference.get();
         numberOfIncompleteFootsteps = walkingFootstepTracker.getNumberOfIncompleteFootsteps();
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
      uiPublisher.publishToUI(BodyPathRegionsForUI, mapRegions);

      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      YoVariableRegistry parentRegistry = new YoVariableRegistry(getClass().getSimpleName());
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
            behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
            autonomousOutput.accept(bodyPathPlanForReview);
         }
      }
      else
      {
         planningFailedTimer.reset();
      }
   }
}
