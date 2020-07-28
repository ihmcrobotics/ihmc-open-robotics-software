package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.Timer;
import us.ihmc.communication.util.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidBehaviors.lookAndStep.*;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walking.WalkingFootstepTracker;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepFootstepPlanningTask
{
   protected StatusLogger statusLogger;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected FootstepPlannerParametersReadOnly footstepPlannerParameters;
   protected Supplier<Boolean> abortGoalWalkingSupplier;
   protected UIPublisher uiPublisher;
   protected Runnable onReachedGoal;
   protected SideDependentList<FramePose3DReadOnly> lastSteppedSolePoses;
   protected FootstepPlanningModule footstepPlanningModule;
   protected Supplier<Boolean> operatorReviewEnabledSupplier;
   protected LookAndStepReview<FootstepPlan> review;
   protected Consumer<FootstepPlan> autonomousOutput;
   protected Runnable planningFailedNotifier;
   protected BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference;
   protected Supplier<Boolean> robotConnectedSupplier;
   protected AtomicReference<RobotSide> lastStanceSideReference;

   public static class LookAndStepFootstepPlanning extends LookAndStepFootstepPlanningTask
   {
      private final SingleThreadSizeOneQueueExecutor executor;
      private final Supplier<RemoteSyncedRobotModel> robotStateSupplier;
      private final WalkingFootstepTracker walkingFootstepTracker;

      private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
      private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
      private Timer planarRegionsExpirationTimer = new Timer();
      private Timer planningFailedTimer = new Timer();
      private BehaviorTaskSuppressor suppressor;

      public LookAndStepFootstepPlanning(StatusLogger statusLogger,
                                         LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                                         FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                         Supplier<Boolean> abortGoalWalkingSupplier,
                                         UIPublisher uiPublisher,
                                         Runnable onReachedGoal,
                                         SideDependentList<FramePose3DReadOnly> lastSteppedSolePoses,
                                         FootstepPlanningModule footstepPlanningModule,
                                         AtomicReference<RobotSide> lastStanceSideReference,
                                         Supplier<Boolean> operatorReviewEnabledSupplier,
                                         RemoteSyncedRobotModel syncedRobot,
                                         BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference,
                                         Supplier<Boolean> robotConnectedSupplier,
                                         WalkingFootstepTracker walkingFootstepTracker)
      {
         this.statusLogger = statusLogger;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.footstepPlannerParameters = footstepPlannerParameters;
         this.abortGoalWalkingSupplier = abortGoalWalkingSupplier;
         this.uiPublisher = uiPublisher;
         this.onReachedGoal = onReachedGoal;
         this.lastSteppedSolePoses = lastSteppedSolePoses;
         this.footstepPlanningModule = footstepPlanningModule;
         this.lastStanceSideReference = lastStanceSideReference;
         this.operatorReviewEnabledSupplier = operatorReviewEnabledSupplier;
         this.robotStateSupplier = () ->
         {
            syncedRobot.update();
            return syncedRobot;
         };
         this.behaviorStateReference = behaviorStateReference;
         this.robotConnectedSupplier = robotConnectedSupplier;
         this.walkingFootstepTracker = walkingFootstepTracker;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         planarRegionsInput.addCallback(data -> runOrQueue());
         bodyPathPlanInput.addCallback(data -> runOrQueue());

         this.planningFailedNotifier = planningFailedTimer::reset;
      }

      public void laterSetup(LookAndStepReview<FootstepPlan> review,
                             Consumer<FootstepPlan> autonomousOutput)
      {
         this.review = review;
         this.autonomousOutput = autonomousOutput;

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Footstep planning");
         suppressor.addCondition("Not in footstep planning state", () -> !behaviorState.equals(LookAndStepBehavior.State.FOOTSTEP_PLANNING));
         suppressor.addCondition(() -> "Regions not OK: " + planarRegions
                                       + ", timePassed: " + planarRegionReceptionTimerSnapshot.getTimePassedSinceReset()
                                       + ", isEmpty: " + (planarRegions == null ? null : planarRegions.isEmpty()),
                                 () -> !regionsOK());
         suppressor.addCondition("Planning failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition(() -> "Body path size: " + (bodyPathPlan == null ? null : bodyPathPlan.size()), () -> !bodyPathPlanOK());
         suppressor.addCondition("Plan being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> !robotConnectedSupplier.get());
         suppressor.addCondition(() -> "numberOfIncompleteFootsteps " + numberOfIncompleteFootsteps
                                       + " > " + lookAndStepBehaviorParameters.getAcceptableIncompleteFootsteps(),
                                 () -> numberOfIncompleteFootsteps > lookAndStepBehaviorParameters.getAcceptableIncompleteFootsteps());
      }

      public void acceptPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
      {
         planarRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
         planarRegionsExpirationTimer.reset();
      }

      public void acceptBodyPathPlan(List<? extends Pose3DReadOnly> bodyPathPlan)
      {
         bodyPathPlanInput.set(bodyPathPlan);
      }

      public synchronized void runOrQueue()
      {
         executor.execute(this::evaluateAndRun);
      }

      private void evaluateAndRun()
      {
         planarRegions = planarRegionsInput.get();
         planarRegionReceptionTimerSnapshot = planarRegionsExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.getPlanarRegionsExpiration());
         planningFailureTimerSnapshot = planningFailedTimer.createSnapshot(lookAndStepBehaviorParameters.getWaitTimeAfterPlanFailed());
         bodyPathPlan = bodyPathPlanInput.get();
         syncedRobot = robotStateSupplier.get();
         lastStanceSide = lastStanceSideReference.get();
         behaviorState = behaviorStateReference.get();
         numberOfIncompleteFootsteps = walkingFootstepTracker.getNumberOfIncompleteFootsteps();

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }
   }

   // instance variables
   protected PlanarRegionsList planarRegions;
   protected TimerSnapshotWithExpiration planarRegionReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration planningFailureTimerSnapshot;
   protected List<? extends Pose3DReadOnly> bodyPathPlan;
   protected RemoteSyncedRobotModel syncedRobot;
   protected RobotSide lastStanceSide;
   protected LookAndStepBehavior.State behaviorState;
   protected int numberOfIncompleteFootsteps;

   protected boolean regionsOK()
   {
      return planarRegions != null && !planarRegions.isEmpty() && planarRegionReceptionTimerSnapshot.isRunning();
   }

   protected boolean bodyPathPlanOK()
   {
      return bodyPathPlan != null && !bodyPathPlan.isEmpty(); // are these null checks necessary?
   }

   protected void performTask()
   {
      statusLogger.info("Finding next sub goal for footstep planning...");
      uiPublisher.publishToUI(MapRegionsForUI, planarRegions);

      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      FramePose3D pelvisPose = new FramePose3D();
      pelvisPose.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D goalPoseBetweenFeet = new FramePose3D();
      goalPoseBetweenFeet.setIncludingFrame(pelvisPose);
      goalPoseBetweenFeet.setZ(midFeetZ);

      // find closest point along body path plan
      Point3D closestPointAlongPath = new Point3D();
      int closestSegmentIndex = BodyPathPlannerTools.findClosestPointAlongPath(bodyPathPlan, goalPoseBetweenFeet.getPosition(), closestPointAlongPath);

      uiPublisher.publishToUI(ClosestPointForUI, new Pose3D(closestPointAlongPath, new Quaternion()));

      // move point along body path plan by plan horizon
      Point3D goalPoint = new Point3D();
      int segmentIndexOfGoal = BodyPathPlannerTools.movePointAlongBodyPath(bodyPathPlan,
                                                                           closestPointAlongPath,
                                                                           goalPoint,
                                                                           closestSegmentIndex,
                                                                           lookAndStepBehaviorParameters.getPlanHorizon());

      Pose3DReadOnly terminalGoal = bodyPathPlan.get(bodyPathPlan.size() - 1);
      boolean reachedGoal = closestPointAlongPath.distanceXY(goalPoint) < lookAndStepBehaviorParameters.getGoalSatisfactionRadius();
      reachedGoal &= Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pelvisPose.getYaw(), terminalGoal.getYaw()))
                     < lookAndStepBehaviorParameters.getGoalSatisfactionOrientationDelta();
      if (reachedGoal | abortGoalWalkingSupplier.get())
      {
         if (reachedGoal) statusLogger.warn("Footstep planning: Robot reached goal. Not planning");
         if (abortGoalWalkingSupplier.get()) statusLogger.warn("Footstep planning: Goal was cancelled. Not planning");
         behaviorStateReference.set(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         onReachedGoal.run();
         return;
      }

      statusLogger.info("Found next sub goal: {}", goalPoint);
      goalPoseBetweenFeet.getPosition().set(goalPoint);
      goalPoseBetweenFeet.getOrientation().set(bodyPathPlan.get(segmentIndexOfGoal + 1).getOrientation());

      // update last stepped poses to plan from; initialize to current poses
      SideDependentList<FramePose3DReadOnly> startFootPoses = new SideDependentList<>();
      ArrayList<FootstepForUI> startFootPosesForUI = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         FramePose3DReadOnly lastSteppedSolePose = lastSteppedSolePoses.get(side);
         if (lastSteppedSolePose == null)
         {
            FramePose3D soleFrameZUpPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleZUpFrame(side));
            soleFrameZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
            lastSteppedSolePose = soleFrameZUpPose;
            lastSteppedSolePoses.put(side, lastSteppedSolePose);
         }

         startFootPosesForUI.add(new FootstepForUI(side, new Pose3D(lastSteppedSolePose), side.getPascalCaseName() + " Start"));
         startFootPoses.put(side, lastSteppedSolePose);
      }
      uiPublisher.publishToUI(StartAndGoalFootPosesForUI, startFootPosesForUI);

      RobotSide stanceSide;
      if (lastStanceSide != null)
      {
         stanceSide = lastStanceSide.getOppositeSide();
      }
      else // if first step, step with furthest foot from the goal
      {
         if (startFootPoses.get(RobotSide.LEFT) .getPosition().distance(goalPoseBetweenFeet.getPosition())
             <= startFootPoses.get(RobotSide.RIGHT).getPosition().distance(goalPoseBetweenFeet.getPosition()))
         {
            stanceSide = RobotSide.LEFT;
         }
         else
         {
            stanceSide = RobotSide.RIGHT;
         }
      }

      lastStanceSideReference.set(stanceSide);

      uiPublisher.publishToUI(SubGoalForUI, new Pose3D(goalPoseBetweenFeet));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(startFootPoses.get(RobotSide.LEFT), startFootPoses.get(RobotSide.RIGHT));
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(planarRegions);
      footstepPlannerRequest.setTimeout(lookAndStepBehaviorParameters.getFootstepPlannerTimeout());
      footstepPlannerRequest.setPerformPositionBasedSplitFractionCalculation(true);
      footstepPlannerRequest.setSwingPlannerType(SwingPlannerType.POSITION);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestPathSize) -> bestPathSize >= lookAndStepBehaviorParameters.getMinimumNumberOfPlannedSteps());

      statusLogger.info("Footstep planner started...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      statusLogger.info("Footstep planner completed with {} steps", footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      // print log duration?
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(30), "FootstepPlanLogDeletion");

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         planningFailedNotifier.run();
      }
      else
      {
         uiPublisher.publishToUI(FootstepPlanForUI, FootstepForUI.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(), "Planned"));
      }

      if (operatorReviewEnabledSupplier.get())
      {
         review.review(footstepPlannerOutput.getFootstepPlan());
      }
      else
      {
         behaviorStateReference.set(LookAndStepBehavior.State.SWINGING);
         autonomousOutput.accept(footstepPlannerOutput.getFootstepPlan());
      }
   }
}
