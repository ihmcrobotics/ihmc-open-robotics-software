package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionCalculator;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commonWalkingControlModules.trajectories.AdaptiveSwingTimingTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.tools.PlannerTools;
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
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.interfaces.UIPublisher;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepFootstepPlanningTask
{
   protected LookAndStepImminentStanceTracker imminentStanceTracker;
   protected BehaviorHelper helper;
   protected StatusLogger statusLogger;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected FootstepPlannerParametersReadOnly footstepPlannerParameters;
   protected SwingPlannerParametersReadOnly swingPlannerParameters;
   protected UIPublisher uiPublisher;
   protected FootstepPlanningModule footstepPlanningModule;
   protected SideDependentList<ConvexPolygon2D> defaultFootPolygons;
   protected Supplier<Boolean> operatorReviewEnabledSupplier;
   protected ROS2SyncedRobotModel syncedRobot;
   protected LookAndStepReview<FootstepPlan> review = new LookAndStepReview<>();
   protected Consumer<FootstepPlan> autonomousOutput;
   protected Timer planningFailedTimer = new Timer();
   protected AtomicReference<RobotSide> lastStanceSideReference;
   protected AtomicReference<Boolean> plannerFailedLastTime = new AtomicReference<>();
   protected YoDouble footholdVolume;

   public static class LookAndStepFootstepPlanning extends LookAndStepFootstepPlanningTask
   {
      // instance variables
      private ResettableExceptionHandlingExecutorService executor;
      private ControllerStatusTracker controllerStatusTracker;
      private Supplier<LookAndStepBehavior.State> behaviorStateReference;

      private final TypedInput<LookAndStepBodyPathLocalizationResult> localizationResultInput = new TypedInput<>();
      private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
      private final TypedInput<PlanarRegionsList> lidarREAPlanarRegionsInput = new TypedInput<>();
      private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
      private final TypedInput<RobotConfigurationData> robotConfigurationDataInput = new TypedInput<>();
      private final Input footstepCompletedInput = new Input();
      private final Timer planarRegionsExpirationTimer = new Timer();
      private final Timer lidarREAPlanarRegionsExpirationTimer = new Timer();
      private final Timer capturabilityBasedStatusExpirationTimer = new Timer();
      private final Timer robotConfigurationDataExpirationTimer = new Timer();
      private BehaviorTaskSuppressor suppressor;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         statusLogger = lookAndStep.statusLogger;
         lookAndStepParameters = lookAndStep.lookAndStepParameters;
         footstepPlannerParameters = lookAndStep.footstepPlannerParameters;
         swingPlannerParameters = lookAndStep.swingPlannerParameters;
         uiPublisher = lookAndStep.helper::publish;
         footstepPlanningModule = lookAndStep.helper.getOrCreateFootstepPlanner();
         defaultFootPolygons = FootstepPlanningModuleLauncher.createFootPolygons(lookAndStep.helper.getRobotModel());
         lastStanceSideReference = lookAndStep.lastStanceSide;
         operatorReviewEnabledSupplier = lookAndStep.operatorReviewEnabledInput::get;
         behaviorStateReference = lookAndStep.behaviorStateReference::get;
         controllerStatusTracker = lookAndStep.controllerStatusTracker;
         imminentStanceTracker = lookAndStep.imminentStanceTracker;
         footholdVolume = new YoDouble("footholdVolume", lookAndStep.yoRegistry);
         helper = lookAndStep.helper;
         autonomousOutput = footstepPlan ->
         {
            if (!lookAndStep.isBeingReset.get())
            {
               lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.STEPPING);
               lookAndStep.stepping.acceptFootstepPlan(footstepPlan);
            }
         };
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         bipedalSupportPlanarRegionCalculator = new BipedalSupportPlanarRegionCalculator(lookAndStep.helper.getRobotModel());

         review.initialize(statusLogger, "footstep plan", lookAndStep.approvalNotification, autonomousOutput);

         executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

         localizationResultInput.addCallback(data -> executor.clearQueueAndExecute(this::evaluateAndRun));
         planarRegionsInput.addCallback(data -> executor.clearQueueAndExecute(this::evaluateAndRun));
         footstepCompletedInput.addCallback(() -> executor.clearQueueAndExecute(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Footstep planning");
         suppressor.addCondition("Not in footstep planning state", () -> !behaviorState.equals(LookAndStepBehavior.State.FOOTSTEP_PLANNING));
         suppressor.addCondition(() -> "Regions expired. haveReceivedAny: " + planarRegionReceptionTimerSnapshot.hasBeenSet()
                                       + " timeSinceLastUpdate: " + planarRegionReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> !lookAndStepParameters.getAssumeFlatGround() && planarRegionReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No regions. "
                                       + (planarRegions == null ? null : (" isEmpty: " + planarRegions.isEmpty())),
                                 () -> !lookAndStepParameters.getAssumeFlatGround() && (!(planarRegions != null && !planarRegions.isEmpty())));
         suppressor.addCondition(() -> "Capturability based status expired. haveReceivedAny: " + capturabilityBasedStatusExpirationTimer.hasBeenSet()
                                       + " timeSinceLastUpdate: " + capturabilityBasedStatusReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> capturabilityBasedStatusReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No capturability based status. ", () -> capturabilityBasedStatus == null);
         suppressor.addCondition(() -> "No localization result. ", () -> localizationResult == null);
         TypedNotification<Boolean> reviewApprovalNotification = lookAndStep.helper.subscribeViaNotification(ReviewApproval);
         Supplier<Boolean> operatorJustRejected = () -> reviewApprovalNotification.poll() && !reviewApprovalNotification.read();
         suppressor.addCondition("Planner failed and operator is reviewing and hasn't just rejected.", () -> plannerFailedLastTime.get()
                                                                                                             && operatorReviewEnabledSupplier.get()
                                                                                                             && !operatorJustRejected.get());
         suppressor.addCondition("Planning failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition("Plan being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> robotDataReceptionTimerSnaphot.isExpired());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
//         suppressor.addCondition(() -> "numberOfIncompleteFootsteps " + numberOfIncompleteFootsteps
//                                       + " > " + lookAndStepParameters.getAcceptableIncompleteFootsteps(),
//                                 () -> numberOfIncompleteFootsteps > lookAndStepParameters.getAcceptableIncompleteFootsteps());
         suppressor.addCondition(() -> "Swing planner type parameter not valid: " + lookAndStepParameters.getSwingPlannerType(),
                                 () -> swingPlannerType == null);
      }

      public void acceptFootstepCompleted()
      {
         footstepCompletedInput.set();
      }

      public void acceptPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
      {
         acceptPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      }

      public void acceptPlanarRegions(PlanarRegionsList planarRegionsList)
      {
         planarRegionsInput.set(planarRegionsList);
         planarRegionsExpirationTimer.reset();
      }

      public void acceptLidarREARegions(PlanarRegionsListMessage lidarREAPlanarRegionsListMessage)
      {
         acceptLidarREARegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(lidarREAPlanarRegionsListMessage));
      }

      public void acceptLidarREARegions(PlanarRegionsList lidarREAPlanarRegionsList)
      {
         lidarREAPlanarRegionsInput.set(lidarREAPlanarRegionsList);
         lidarREAPlanarRegionsExpirationTimer.reset();
      }

      public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
      {
         capturabilityBasedStatusInput.set(capturabilityBasedStatus);
         capturabilityBasedStatusExpirationTimer.reset();
      }

      public void acceptRobotConfigurationData(RobotConfigurationData robotConfigurationData)
      {
         robotConfigurationDataInput.set(robotConfigurationData);
         robotConfigurationDataExpirationTimer.reset();
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
         planarRegionsHistory.clear();
      }

      private void evaluateAndRun()
      {
         planarRegions = planarRegionsInput.getLatest();
         lidarREAPlanarRegions = lidarREAPlanarRegionsInput.getLatest();
         planarRegionReceptionTimerSnapshot = planarRegionsExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         lidarREAPlanarRegionReceptionTimerSnapshot = lidarREAPlanarRegionsExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
         capturabilityBasedStatusReceptionTimerSnapshot
               = capturabilityBasedStatusExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         robotConfigurationData = robotConfigurationDataInput.getLatest();
         robotConfigurationDataReceptionTimerSnapshot
               = robotConfigurationDataExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
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

      public void destroy()
      {
         executor.destroy();
      }
   }

   // snapshot data
   protected LookAndStepBodyPathLocalizationResult localizationResult;
   protected PlanarRegionsList planarRegions;
   protected PlanarRegionsList lidarREAPlanarRegions;
   protected BipedalSupportPlanarRegionCalculator bipedalSupportPlanarRegionCalculator;
   protected ArrayDeque<PlanarRegionsList> planarRegionsHistory = new ArrayDeque<>();
   protected CapturabilityBasedStatus capturabilityBasedStatus;
   protected RobotConfigurationData robotConfigurationData;
   protected TimerSnapshotWithExpiration planarRegionReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration lidarREAPlanarRegionReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration capturabilityBasedStatusReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration robotConfigurationDataReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration planningFailureTimerSnapshot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected RobotSide lastStanceSide;
   protected LookAndStepBehavior.State behaviorState;
   protected int numberOfIncompleteFootsteps;
   protected int numberOfCompletedFootsteps;
   protected SwingPlannerType swingPlannerType;

   protected void performTask()
   {
      if (planarRegionsHistory.isEmpty()
       && lookAndStepParameters.getPlanarRegionsHistorySize() > 0
       && lookAndStepParameters.getUseInitialSupportRegions()
       && capturabilityBasedStatusReceptionTimerSnapshot.isRunning()
       && robotConfigurationDataReceptionTimerSnapshot.isRunning())
      {
         bipedalSupportPlanarRegionCalculator.calculateSupportRegions(lookAndStepParameters.getSupportRegionScaleFactor(),
                                                                      capturabilityBasedStatus,
                                                                      robotConfigurationData);
         planarRegionsHistory.addLast(bipedalSupportPlanarRegionCalculator.getSupportRegionsAsList());
      }

      // detect flat ground; work in progress
      ConvexPolytope3D convexPolytope = new ConvexPolytope3D();
      for (Point3D point3D : capturabilityBasedStatus.getLeftFootSupportPolygon3d())
      {
         convexPolytope.addVertex(point3D);
      }
      for (Point3D point3D : capturabilityBasedStatus.getRightFootSupportPolygon3d())
      {
         convexPolytope.addVertex(point3D);
      }
      footholdVolume.set(convexPolytope.getVolume());

      if (lookAndStepParameters.getAssumeFlatGround())
      {
         SideDependentList<Boolean> isInSupport = new SideDependentList<>(!capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty(),
                                                                          !capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty());
         boolean bothInSupport = isInSupport.get(RobotSide.LEFT) && isInSupport.get(RobotSide.RIGHT);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         if (bothInSupport)
         {
            FramePose3D leftSole = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
            FramePose3D rightSole = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
            leftSole.changeFrame(ReferenceFrame.getWorldFrame());
            rightSole.changeFrame(ReferenceFrame.getWorldFrame());
            FramePose3D midFeetPose = new FramePose3D();
            midFeetPose.set(leftSole);
            midFeetPose.getPosition().interpolate(rightSole.getPosition(), 0.5);
            midFeetPose.getOrientation().setToZero();
            midFeetPose.get(transformToWorld);
         }
         else
         {
            for (RobotSide side : RobotSide.values)
            {
               if (isInSupport.get(side))
               {
                  FramePose3D supportSole = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(side));
                  FramePose3D otherSole = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(side.getOppositeSide()));
                  supportSole.changeFrame(ReferenceFrame.getWorldFrame());
                  otherSole.changeFrame(ReferenceFrame.getWorldFrame());
                  FramePose3D midFeetPose = new FramePose3D();
                  midFeetPose.set(supportSole);
                  Vector3D normal = new Vector3D(Axis3D.Z);
                  midFeetPose.getOrientation().transform(normal);
                  Plane3D plane = new Plane3D(midFeetPose.getPosition(), normal);
                  Point3D projectedOtherSolePosition = EuclidGeometryTools.orthogonalProjectionOnPlane3D(otherSole.getPosition(),
                                                                                                         plane.getPoint(),
                                                                                                         plane.getNormal());
                  midFeetPose.getPosition().interpolate(projectedOtherSolePosition, 0.5);
                  midFeetPose.getOrientation().setToZero();
                  midFeetPose.get(transformToWorld);
               }
            }
         }
         ArrayList<ConvexPolygon2D> polygons = new ArrayList<>();
         ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
         double radius = lookAndStepParameters.getAssumedFlatGroundCircleRadius();
         for (int i = 0; i < 40; i++)
         {
            double angle = (i / 40.0) * 2.0 * Math.PI;
            convexPolygon2D.addVertex(radius * Math.cos(angle), radius * Math.sin(angle));
         }
         convexPolygon2D.update();
         polygons.add(convexPolygon2D);
         PlanarRegion circleRegion = new PlanarRegion(transformToWorld, polygons);
         PlanarRegionsList planarRegionsList = new PlanarRegionsList();
         planarRegionsList.addPlanarRegion(circleRegion);
         planarRegionsHistory.addLast(planarRegionsList);
      }
      else
      {
         planarRegionsHistory.add(planarRegions);
      }

      PlanarRegionsList combinedRegionsForPlanning = new PlanarRegionsList();
      planarRegionsHistory.forEach(combinedRegionsForPlanning::addPlanarRegionsList);

      uiPublisher.publishToUI(PlanarRegionsForUI, combinedRegionsForPlanning);

      Point3D closestPointAlongPath = localizationResult.getClosestPointAlongPath();
      int closestSegmentIndex = localizationResult.getClosestSegmentIndex();
      List<? extends Pose3DReadOnly> bodyPathPlan = localizationResult.getBodyPathPlan();
      SideDependentList<MinimalFootstep> startFootPoses = imminentStanceTracker.calculateImminentStancePoses();

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

      // calculate impassibility
      if (lookAndStepParameters.getStopForImpassibilities() && lidarREAPlanarRegions != null)
      {
         Pose3D rootPose = new Pose3D(new Point3D(robotConfigurationData.getRootTranslation()), robotConfigurationData.getRootOrientation());
         BodyCollisionData collisionData = PlannerTools.detectCollisionsAlongBodyPath(rootPose,
                                                                                      bodyPathPlan,
                                                                                      lidarREAPlanarRegions,
                                                                                      footstepPlannerParameters,
                                                                                      lookAndStepParameters.getHorizonFromDebrisToStop());
         if (collisionData != null && collisionData.isCollisionDetected())
         {
            uiPublisher.publishToUI(Obstacle, MutablePair.of(new Pose3D(collisionData.getBodyBox().getPose()),
                                                             new Vector3D(collisionData.getBodyBox().getSize())));
            uiPublisher.publishToUI(ImpassibilityDetected, true);
            doFailureAction("Impassibility detected. Aborting task...");
            return;
         }
      }
      uiPublisher.publishToUI(ImpassibilityDetected, false);

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
      // if last plan failed
      // if foot is in the air
      // if how many steps are left

      if (lastStanceSide != null)
      {
         // if planner failed last time, do not switch sides
//         stanceSide = plannerFailedLastTime.get() ? lastStanceSide : lastStanceSide.getOppositeSide();
         // Actually look and step can get stuck if you don't switch sides
         stanceSide = lastStanceSide.getOppositeSide();
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
      // TODO: Need to plan from where current stance and swing end are to prevent re-stepping in the same spot
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), subGoalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(combinedRegionsForPlanning);
      footstepPlannerRequest.setTimeout(lookAndStepParameters.getFootstepPlannerTimeout());
      footstepPlannerRequest.setSwingPlannerType(swingPlannerType);
      footstepPlannerRequest.setSnapGoalSteps(true);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.getSwingPlanningModule().getSwingPlannerParameters().set(swingPlannerParameters);
      footstepPlanningModule.clearCustomTerminationConditions();
      footstepPlanningModule.addCustomTerminationCondition(
            (plannerTime, iterations, bestPathFinalStep, bestSecondToFinalStep, bestPathSize) -> bestPathSize >= lookAndStepParameters.getNumberOfStepsToTryToPlan());
      MinimumFootstepChecker stepInPlaceChecker = new MinimumFootstepChecker();
      stepInPlaceChecker.setStanceFeetPoses(startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld(), startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld());
      footstepPlanningModule.getChecker().clearCustomFootstepCheckers();
      footstepPlanningModule.getChecker().attachCustomFootstepChecker(stepInPlaceChecker);

      statusLogger.info("Stance side: {}", stanceSide.name());
      statusLogger.info("Planning footsteps with {}...", swingPlannerType.name());
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      statusLogger.info("Footstep planner completed with {}, {} step(s)",
                        footstepPlannerOutput.getFootstepPlanningResult(),
                        footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      // print log duration?
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      uiPublisher.publishToUI(FootstepPlannerLatestLogPath, footstepPlannerLogger.getLatestLogDirectory());
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

      // TODO: Detect step down and reject unless we planned two steps.
      // Should get closer to the edge somehow?  Solve this in the footstep planner?
      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanningModule);
         rejectionReasonReport.update();
         ArrayList<Pair<Integer, Double>> rejectionReasonsMessage = new ArrayList<>();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            statusLogger.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
            rejectionReasonsMessage.add(MutablePair.of(reason.ordinal(), MathTools.roundToSignificantFigures(rejectionPercentage, 3)));
         }
         uiPublisher.publishToUI(FootstepPlannerRejectionReasons, rejectionReasonsMessage);

         doFailureAction("Footstep planning failure. Aborting task...");
      }
      else
      {
         while (planarRegionsHistory.size() > lookAndStepParameters.getPlanarRegionsHistorySize())
         {
            planarRegionsHistory.removeFirst();
         }
         uiPublisher.publishToUI(PlannedFootstepsForUI, MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(), "Planned"));

         FootstepPlan footstepPlan = new FootstepPlan();
         for (int i = 0; i < lookAndStepParameters.getMaxStepsToSendToController()
                         && i < footstepPlannerOutput.getFootstepPlan().getNumberOfSteps(); i++)
         {
            footstepPlan.addFootstep(new PlannedFootstep(footstepPlannerOutput.getFootstepPlan().getFootstep(i)));
         }
         footstepPlan.setFinalTransferSplitFraction(footstepPlannerOutput.getFootstepPlan().getFinalTransferSplitFraction());
         footstepPlan.setFinalTransferWeightDistribution(footstepPlannerOutput.getFootstepPlan().getFinalTransferWeightDistribution());

         // Extend the swing duration if necessary.
         // TODO: Check and see if this is ensured by the footstep planner and remove it.
         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
         {
            PlannedFootstep footstep = footstepPlan.getFootstep(i);
            Pose3DReadOnly startStep;
            if (i == 0)
            {
               startStep = startFootPoses.get(footstep.getRobotSide()).getSolePoseInWorld();
            }
            else
            {
               startStep = footstepPlan.getFootstep(i - 1).getFootstepPose();
            }
            double idealStepLength = footstepPlannerParameters.getIdealFootstepLength();
            double maxStepZ = footstepPlannerParameters.getMaxStepZ();
            double calculatedSwing = AdaptiveSwingTimingTools.calculateSwingTime(idealStepLength,
                                                                                 footstepPlannerParameters.getMaxSwingReach(),
                                                                                 maxStepZ,
                                                                                 swingPlannerParameters.getMinimumSwingTime(),
                                                                                 swingPlannerParameters.getMaximumSwingTime(),
                                                                                 startStep.getPosition(),
                                                                                 footstep.getFootstepPose().getPosition());
            if (footstep.getSwingDuration() < calculatedSwing)
            {
               statusLogger.info("Increasing swing duration to {} s", calculatedSwing);
               footstep.setSwingDuration(calculatedSwing);
            }
            footstep.setTransferDuration(lookAndStepParameters.getTransferDuration()); // But probably keep this.
         }

         if (operatorReviewEnabledSupplier.get())
         {
            helper.getOrCreateRobotInterface().pauseWalking();
            review.review(footstepPlan);
         }
         else
         {
            autonomousOutput.accept(footstepPlan);
         }
      }
   }

   private void doFailureAction(String message)
   {
      // Finish the currently swinging step and stop walking
      helper.getOrCreateRobotInterface().pauseWalking();

      if (!planarRegionsHistory.isEmpty())
         planarRegionsHistory.removeLast();

      statusLogger.info(message);
      plannerFailedLastTime.set(true);
      planningFailedTimer.reset();
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
