package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.CustomFootstepChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepPlanEtcetera;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.*;
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
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected FootstepPlannerParametersReadOnly footstepPlannerParameters;
   protected SwingPlannerParametersReadOnly swingPlannerParameters;
   protected UIPublisher uiPublisher;
   protected FootstepPlanningModule footstepPlanningModule;
   protected SideDependentList<ConvexPolygon2D> defaultFootPolygons;
   protected Supplier<Boolean> operatorReviewEnabledSupplier;
   protected RemoteSyncedRobotModel syncedRobot;
   protected LookAndStepReview<FootstepPlanEtcetera> review = new LookAndStepReview<>();
   protected Consumer<FootstepPlanEtcetera> autonomousOutput;
   protected Runnable planningFailedNotifier;
   protected AtomicReference<RobotSide> lastStanceSideReference;
   protected AtomicReference<Boolean> plannerFailedLastTime = new AtomicReference<>();

   public static class LookAndStepFootstepPlanning extends LookAndStepFootstepPlanningTask
   {
      // instance variables
      private SingleThreadSizeOneQueueExecutor executor;
      private ControllerStatusTracker controllerStatusTracker;
      private Supplier<LookAndStepBehavior.State> behaviorStateReference;

      private final TypedInput<LookAndStepBodyPathLocalizationResult> localizationResultInput = new TypedInput<>();
      private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
      private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
      private final Input footstepCompletedInput = new Input();
      private final Timer planarRegionsExpirationTimer = new Timer();
      private final Timer capturabilityBasedStatusExpirationTimer = new Timer();
      private final Timer planningFailedTimer = new Timer();
      private BehaviorTaskSuppressor suppressor;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         statusLogger = lookAndStep.statusLogger;
         lookAndStepParameters = lookAndStep.lookAndStepParameters;
         footstepPlannerParameters = lookAndStep.footstepPlannerParameters;
         swingPlannerParameters = lookAndStep.swingPlannerParameters;
         uiPublisher = lookAndStep.helper::publishToUI;
         footstepPlanningModule = lookAndStep.helper.getOrCreateFootstepPlanner();
         defaultFootPolygons = FootstepPlanningModuleLauncher.createFootPolygons(lookAndStep.helper.getRobotModel());
         lastStanceSideReference = lookAndStep.lastStanceSide;
         operatorReviewEnabledSupplier = lookAndStep.operatorReviewEnabledInput::get;
         behaviorStateReference = lookAndStep.behaviorStateReference::get;
         controllerStatusTracker = lookAndStep.controllerStatusTracker;
         autonomousOutput = footstepPlanEtc ->
         {
            if (!lookAndStep.isBeingReset.get())
            {
               lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.STEPPING);
               lookAndStep.stepping.acceptFootstepPlan(footstepPlanEtc);
            }
         };
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();

         review.initialize(statusLogger, "footstep plan", lookAndStep.approvalNotification, autonomousOutput);

         planningFailedNotifier = planningFailedTimer::reset;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         localizationResultInput.addCallback(data -> executor.submitTask(this::evaluateAndRun));
         planarRegionsInput.addCallback(data -> executor.submitTask(this::evaluateAndRun));
         footstepCompletedInput.addCallback(() -> executor.submitTask(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Footstep planning");
         suppressor.addCondition("Not in footstep planning state", () -> !behaviorState.equals(LookAndStepBehavior.State.FOOTSTEP_PLANNING));
         suppressor.addCondition(() -> "Regions expired. haveReceivedAny: " + planarRegionReceptionTimerSnapshot.hasBeenSet()
                                       + " timeSinceLastUpdate: " + planarRegionReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> planarRegionReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No regions. "
                                       + (planarRegions == null ? null : (" isEmpty: " + planarRegions.isEmpty())),
                                 () -> !(planarRegions != null && !planarRegions.isEmpty()));
         suppressor.addCondition(() -> "Capturability based status expired. haveReceivedAny: " + capturabilityBasedStatusExpirationTimer.hasBeenSet()
                                       + " timeSinceLastUpdate: " + capturabilityBasedStatusReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> capturabilityBasedStatusReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No capturability based status. ", () -> capturabilityBasedStatus == null);
         suppressor.addCondition(() -> "No localization result. ", () -> localizationResult == null);
         suppressor.addCondition("Planning failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition("Plan being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> robotDataReceptionTimerSnaphot.isExpired());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
         suppressor.addCondition(() -> "numberOfIncompleteFootsteps " + numberOfIncompleteFootsteps
                                       + " > " + lookAndStepParameters.getAcceptableIncompleteFootsteps(),
                                 () -> numberOfIncompleteFootsteps > lookAndStepParameters.getAcceptableIncompleteFootsteps());
         suppressor.addCondition(() -> "Swing planner type parameter not valid: " + lookAndStepParameters.getSwingPlannerType(),
                                 () -> swingPlannerType == null);
      }

      public void acceptFootstepCompleted()
      {
         footstepCompletedInput.set();
      }

      public void acceptPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
      {
         planarRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
         planarRegionsExpirationTimer.reset();
      }

      public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
      {
         capturabilityBasedStatusInput.set(capturabilityBasedStatus);
         capturabilityBasedStatusExpirationTimer.reset();
      }

      public void acceptLocalizationResult(LookAndStepBodyPathLocalizationResult localizationResult)
      {
         localizationResultInput.set(localizationResult);
      }

      public void reset()
      {
         executor.interruptAndReset();
         review.reset();
         plannerFailedLastTime.set(false);
      }

      private void evaluateAndRun()
      {
         planarRegions = planarRegionsInput.getLatest();
         planarRegionReceptionTimerSnapshot = planarRegionsExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
         capturabilityBasedStatusReceptionTimerSnapshot
               = capturabilityBasedStatusExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         planningFailureTimerSnapshot = planningFailedTimer.createSnapshot(lookAndStepParameters.getWaitTimeAfterPlanFailed());
         localizationResult = localizationResultInput.getLatest();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepParameters.getRobotConfigurationDataExpiration());
         lastStanceSide = lastStanceSideReference.get();
         behaviorState = behaviorStateReference.get();
         numberOfIncompleteFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps();
         numberOfCompletedFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfCompletedFootsteps();
         swingPlannerType = SwingPlannerType.fromInt(lookAndStepParameters.getSwingPlannerType());

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }
   }

   // snapshot data
   protected LookAndStepBodyPathLocalizationResult localizationResult;
   protected PlanarRegionsList planarRegions;
   protected CapturabilityBasedStatus capturabilityBasedStatus;
   protected TimerSnapshotWithExpiration planarRegionReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration capturabilityBasedStatusReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration planningFailureTimerSnapshot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected RobotSide lastStanceSide;
   protected LookAndStepBehavior.State behaviorState;
   protected int numberOfIncompleteFootsteps;
   protected int numberOfCompletedFootsteps;
   protected SwingPlannerType swingPlannerType;

   protected void performTask()
   {
      uiPublisher.publishToUI(PlanarRegionsForUI, planarRegions);

      Point3D closestPointAlongPath = localizationResult.getClosestPointAlongPath();
      int closestSegmentIndex = localizationResult.getClosestSegmentIndex();
      Pose3DReadOnly midFeetAlongPath = localizationResult.getMidFeetAlongPath();
      List<? extends Pose3DReadOnly> bodyPathPlan = localizationResult.getBodyPathPlan();
      SideDependentList<MinimalFootstep> startFootPoses = localizationResult.getStanceForPlanning();

      // move point along body path plan by plan horizon
      Pose3D subGoalPoseBetweenFeet = new Pose3D();
      int segmentIndexOfGoal = BodyPathPlannerTools.movePointAlongBodyPath(bodyPathPlan,
                                                                           closestPointAlongPath,
                                                                           subGoalPoseBetweenFeet.getPosition(),
                                                                           closestSegmentIndex,
                                                                           lookAndStepParameters.getPlanHorizon());

      statusLogger.info("Found next sub goal: {}", subGoalPoseBetweenFeet);
      // TODO: Calculate orientation based on a trajectory
      subGoalPoseBetweenFeet.getOrientation().set(bodyPathPlan.get(segmentIndexOfGoal + 1).getOrientation());

      // update last stepped poses to plan from; initialize to current poses
      ArrayList<MinimalFootstep> startFootPosesForUI = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         startFootPosesForUI.add(new MinimalFootstep(side,
                                                     new Pose3D(startFootPoses.get(side).getSolePoseInWorld()),
                                                     startFootPoses.get(side).getFoothold(),
                                                     side.getPascalCaseName() + " Start"));
      }
      uiPublisher.publishToUI(StartAndGoalFootPosesForUI, startFootPosesForUI);

      RobotSide stanceSide;
      if (lastStanceSide != null)
      {
         // if planner failed last time, do not switch sides
         stanceSide = plannerFailedLastTime.get() ? lastStanceSide : lastStanceSide.getOppositeSide();
      }
      else // if first step, step with furthest foot from the goal
      {
         if (startFootPoses.get(RobotSide.LEFT ).getSolePoseInWorld().getPosition().distance(subGoalPoseBetweenFeet.getPosition())
          <= startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld().getPosition().distance(subGoalPoseBetweenFeet.getPosition()))
         {
            stanceSide = RobotSide.LEFT;
         }
         else
         {
            stanceSide = RobotSide.RIGHT;
         }
      }
      plannerFailedLastTime.set(false);

      lastStanceSideReference.set(stanceSide);

      uiPublisher.publishToUI(SubGoalForUI, new Pose3D(subGoalPoseBetweenFeet));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld(),
                                               startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld());
      // TODO: Set start footholds!!
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), subGoalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(planarRegions);
      footstepPlannerRequest.setTimeout(lookAndStepParameters.getFootstepPlannerTimeout());
      footstepPlannerRequest.setSwingPlannerType(swingPlannerType);
      footstepPlannerRequest.setSnapGoalSteps(true);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.getPostProcessHandler().getSwingPlannerParameters().set(swingPlannerParameters);
      footstepPlanningModule.addCustomTerminationCondition(
            (plannerTime, iterations, bestPathFinalStep, bestSecondToFinalStep, bestPathSize) -> bestPathSize >= lookAndStepParameters.getMinimumNumberOfPlannedSteps());
      MinimumFootstepChecker stepInPlaceChecker = new MinimumFootstepChecker();
      stepInPlaceChecker.setStanceFeetPoses(startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld(), startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld());
      footstepPlanningModule.getChecker().attachCustomFootstepChecker(stepInPlaceChecker);

      statusLogger.info("Planning footsteps with {}...", swingPlannerType.name());
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      statusLogger.info("Footstep planner completed with {}, {} step(s)",
                        footstepPlannerOutput.getFootstepPlanningResult(),
                        footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      // print log duration?
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanningModule);
         rejectionReasonReport.update();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            statusLogger.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
         }

         statusLogger.info("Footstep planning failure. Aborting task...");
         plannerFailedLastTime.set(true);
         planningFailedNotifier.run();
      }
      else
      {
         uiPublisher.publishToUI(FootstepPlanForUI, MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(), "Planned"));

         FootstepPlanEtcetera footstepPlanEtc = new FootstepPlanEtcetera(footstepPlannerOutput.getFootstepPlan(),
                                                                         planarRegions,
                                                                         new SideDependentList<>(startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld(),
                                                                                                 startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld()),
                                                                         defaultFootPolygons,
                                                                         swingPlannerType);

         if (operatorReviewEnabledSupplier.get())
         {
            review.review(footstepPlanEtc);
         }
         else
         {
            autonomousOutput.accept(footstepPlanEtc);
         }
      }
   }

   private class MinimumFootstepChecker implements CustomFootstepChecker
   {
      private final Pose3D leftFootStancePose = new Pose3D();
      private final Pose3D rightFootStancePose = new Pose3D();

      private double minimumTranslation = lookAndStepParameters.getMinimumStepTranslation();
      private double minimumRotation = Math.toRadians(lookAndStepParameters.getMinimumStepOrientation());

      public void setStanceFeetPoses(SideDependentList<Pose3DReadOnly> stanceFeetPoses)
      {
         setStanceFeetPoses(stanceFeetPoses.get(RobotSide.LEFT), stanceFeetPoses.get(RobotSide.RIGHT));
      }

      public void setStanceFeetPoses(Pose3DReadOnly leftFootStancePose, Pose3DReadOnly rightFootStancePose)
      {
         this.leftFootStancePose.set(leftFootStancePose);
         this.rightFootStancePose.set(rightFootStancePose);
      }

      public void setMinimumTranslation(double minimumTranslation)
      {
         this.minimumTranslation = minimumTranslation;
      }

      public void setMinimumRotation(double minimumRotation)
      {
         this.minimumRotation = minimumRotation;
      }

      @Override
      public boolean isStepValid(DiscreteFootstep candidateFootstep, DiscreteFootstep stanceNode)
      {
         double distanceX, distanceY, angularDistance;
         if (candidateFootstep.getRobotSide() == RobotSide.LEFT)
         {
            distanceX = leftFootStancePose.getX() - candidateFootstep.getX();
            distanceY = leftFootStancePose.getY() - candidateFootstep.getY();
            angularDistance = AngleTools.computeAngleDifferenceMinusPiToPi(leftFootStancePose.getYaw(), candidateFootstep.getYaw());
         }
         else
         {
            distanceX = rightFootStancePose.getX() - candidateFootstep.getX();
            distanceY = rightFootStancePose.getY() - candidateFootstep.getY();
            angularDistance = AngleTools.computeAngleDifferenceMinusPiToPi(rightFootStancePose.getYaw(), candidateFootstep.getYaw());
         }

         return EuclidCoreTools.norm(distanceX, distanceY) > minimumTranslation || angularDistance > minimumRotation;
      }

      @Override
      public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
      {
         return BipedalFootstepPlannerNodeRejectionReason.STEP_IN_PLACE;
      }
   }
}
