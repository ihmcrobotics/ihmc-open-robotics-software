package us.ihmc.behaviors.lookAndStep;

import behavior_msgs.msg.dds.MinimalFootstepListMessage;
import behavior_msgs.msg.dds.MinimalFootstepMessage;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.Box3DMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage;
import toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.interfaces.UIPublisher;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.trajectories.AdaptiveSwingTimingTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.CustomFootstepChecker;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

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
   protected Timer successfulPlanExpirationTimer = new Timer();
   protected AtomicReference<Boolean> plannerFailedLastTime = new AtomicReference<>();
   protected double footholdVolume;
   protected double planarRegionDelay;
   protected double footstepPlanningDuration;
   protected double moreInclusivePlanningDuration;
   protected FootstepPlan previousFootstepPlan = null;

   public static class LookAndStepFootstepPlanning extends LookAndStepFootstepPlanningTask
   {
      // instance variables
      private ResettableExceptionHandlingExecutorService executor;
      protected ControllerStatusTracker controllerStatusTracker;
      private Supplier<LookAndStepBehavior.State> behaviorStateReference;
      private ROS2StoredPropertySet<LookAndStepBehaviorParametersBasics> ros2LookAndStepParameters;
      private ROS2StoredPropertySet<FootstepPlannerParametersBasics> ros2FootstepPlannerParameters;
      private ROS2StoredPropertySet<SwingPlannerParametersBasics> ros2SwingPlannerParameters;

      private final TypedInput<LookAndStepBodyPathLocalizationResult> localizationResultInput = new TypedInput<>();
      private final TypedInput<PlanarRegionsList> lidarREAPlanarRegionsInput = new TypedInput<>();
      private final TypedInput<HeightMapData> heightMapInput = new TypedInput<>();
      private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
      private final TypedInput<RobotConfigurationData> robotConfigurationDataInput = new TypedInput<>();
      private final Input footstepCompletedInput = new Input();
      private final Timer planarRegionsExpirationTimer = new Timer();
      private final Timer heightMapExpirationTimer = new Timer();
      private final Timer lidarREAPlanarRegionsExpirationTimer = new Timer();
      private final Timer capturabilityBasedStatusExpirationTimer = new Timer();
      private final Timer robotConfigurationDataExpirationTimer = new Timer();
      private BehaviorTaskSuppressor suppressor;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         statusLogger = lookAndStep.statusLogger;
         ros2LookAndStepParameters = lookAndStep.ros2LookAndStepParameters;
         lookAndStepParameters = ros2LookAndStepParameters.getStoredPropertySet();
         ros2FootstepPlannerParameters = lookAndStep.ros2FootstepPlannerParameters;
         footstepPlannerParameters = ros2FootstepPlannerParameters.getStoredPropertySet();
         ros2SwingPlannerParameters = lookAndStep.ros2SwingPlannerParameters;
         swingPlannerParameters = ros2SwingPlannerParameters.getStoredPropertySet();
         uiPublisher = lookAndStep.helper::publish;
         footstepPlanningModule = lookAndStep.helper.getOrCreateFootstepPlanner();
         defaultFootPolygons = FootstepPlanningModuleLauncher.createFootPolygons(lookAndStep.helper.getRobotModel());
         operatorReviewEnabledSupplier = lookAndStep.operatorReviewEnabledInput::get;
         behaviorStateReference = lookAndStep.behaviorStateReference::get;
         controllerStatusTracker = lookAndStep.controllerStatusTracker;
         imminentStanceTracker = lookAndStep.imminentStanceTracker;
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
         planarRegionsManager = new LookAndStepPlanarRegionsManager(lookAndStepParameters, lookAndStep.helper.getRobotModel(), syncedRobot);

         review.initialize(statusLogger, "footstep plan", lookAndStep.approvalNotification, autonomousOutput);

         executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

         localizationResultInput.addCallback(data -> executor.clearQueueAndExecute(this::evaluateAndRun));
         planarRegionsManager.addCallback(data -> executor.clearQueueAndExecute(this::evaluateAndRun));
         footstepCompletedInput.addCallback(() -> executor.clearQueueAndExecute(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Footstep planning");
         suppressor.addCondition("Not in footstep planning state", () -> !behaviorState.equals(LookAndStepBehavior.State.FOOTSTEP_PLANNING));
         suppressor.addCondition(() -> "Environment model expired. haveReceivedAnyRegions: " + planarRegionReceptionTimerSnapshot.hasBeenSet() +
                                       " haveReceivedHeightMap: " + heightMapReceptionTimerSnapshot.hasBeenSet() +
                                       " timeSinceLastRegionsUpdate: " + planarRegionReceptionTimerSnapshot.getTimePassedSinceReset() +
                                       " timeSinceLastHeightMap: " + heightMapReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> !lookAndStepParameters.getAssumeFlatGround() && planarRegionReceptionTimerSnapshot.isExpired() && heightMapReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No regions. " + (planarRegionsManager.getReceivedPlanarRegions() == null ?
                                       null :
                                       (" isEmpty: " + planarRegionsManager.getReceivedPlanarRegions().isEmpty())),
                                 () -> !lookAndStepParameters.getAssumeFlatGround() && (!(planarRegionsManager.getReceivedPlanarRegions() != null
                                                                                          && !planarRegionsManager.getReceivedPlanarRegions().isEmpty())));
         suppressor.addCondition(() -> "Capturability based status expired. haveReceivedAny: " + capturabilityBasedStatusExpirationTimer.hasBeenSet()
                                       + " timeSinceLastUpdate: " + capturabilityBasedStatusReceptionTimerSnapshot.getTimePassedSinceReset(),
                                 () -> capturabilityBasedStatusReceptionTimerSnapshot.isExpired());
         suppressor.addCondition(() -> "No capturability based status. ", () -> capturabilityBasedStatus == null);
         suppressor.addCondition(() -> "No localization result. ", () -> localizationResult == null);
         TypedNotification<Boolean> reviewApprovalNotification = lookAndStep.helper.subscribeViaBooleanNotification(REVIEW_APPROVAL);
         Supplier<Boolean> operatorJustRejected = () -> reviewApprovalNotification.poll() && !reviewApprovalNotification.read();
         suppressor.addCondition("Planner failed and operator is reviewing and hasn't just rejected.",
                                 () -> plannerFailedLastTime.get() && operatorReviewEnabledSupplier.get() && !operatorJustRejected.get());
         suppressor.addCondition("Planning failed recently", () -> planningFailureTimerSnapshot.isRunning());
         suppressor.addCondition("Plan being reviewed", review::isBeingReviewed);
         suppressor.addCondition("Robot disconnected", () -> robotDataReceptionTimerSnaphot.isExpired());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
         suppressor.addCondition(() -> "numberOfIncompleteFootsteps " + numberOfIncompleteFootsteps + " > "
                                       + lookAndStepParameters.getAcceptableIncompleteFootsteps(),
                                 () -> lookAndStepParameters.getMaxStepsToSendToController() == 1
                                       && numberOfIncompleteFootsteps > lookAndStepParameters.getAcceptableIncompleteFootsteps());
         suppressor.addCondition(() -> "Swing planner type parameter not valid: " + lookAndStepParameters.getSwingPlannerType(),
                                 () -> swingPlannerType == null);
         suppressor.addCondition("Planner is current running.", () -> footstepPlanningModule.isPlanning());
      }

      public void acceptFootstepCompleted()
      {
         footstepCompletedInput.set();
      }

      public void acceptFootstepStarted(FootstepStatusMessage footstepStatusMessage)
      {
         stepsStartedWhilePlanning.add(footstepStatusMessage);
      }

      public void acceptHeightMap(HeightMapMessage heightMapMessage)
      {
         heightMapExpirationTimer.reset();
         heightMapInput.set(HeightMapMessageTools.unpackMessage(heightMapMessage));
      }

      public void acceptPlanarRegions(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
      {
         planarRegionDelay = TimeTools.calculateDelay(framePlanarRegionsListMessage.getPlanarRegions().getLastUpdated().getSecondsSinceEpoch(),
                                                      framePlanarRegionsListMessage.getPlanarRegions().getLastUpdated().getAdditionalNanos());
         acceptPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(framePlanarRegionsListMessage));
      }

      public void acceptPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
      {
         planarRegionDelay = TimeTools.calculateDelay(planarRegionsListMessage.getLastUpdated().getSecondsSinceEpoch(),
                                                      planarRegionsListMessage.getLastUpdated().getAdditionalNanos());
         acceptPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      }

      public void acceptPlanarRegions(PlanarRegionsList planarRegionsList)
      {
         helper.publish(RECEIVED_PLANAR_REGIONS_FOR_UI, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));

         planarRegionsExpirationTimer.reset();

         planarRegionsManager.acceptPlanarRegions(planarRegionsList);
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

         planarRegionsManager.acceptCapturabilityBasedStatus(capturabilityBasedStatus);
      }

      public void acceptRobotConfigurationData(RobotConfigurationData robotConfigurationData)
      {
         robotConfigurationDataInput.set(robotConfigurationData);
         robotConfigurationDataExpirationTimer.reset();

         planarRegionsManager.acceptRobotConfigurationData(robotConfigurationData);
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
         planarRegionsManager.clear();
         lastPlanInitialStanceSide = null;
      }

      private void evaluateAndRun()
      {
         ros2LookAndStepParameters.update();
         ros2FootstepPlannerParameters.update();
         ros2SwingPlannerParameters.update();
         lidarREAPlanarRegions = lidarREAPlanarRegionsInput.getLatest();
         heightMapData = heightMapInput.getLatest();
         planarRegionReceptionTimerSnapshot = planarRegionsExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
         heightMapReceptionTimerSnapshot = heightMapExpirationTimer.createSnapshot(lookAndStepParameters.getHeightMapExpiration());
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
         RobotSide lastStartedRobotSide = imminentStanceTracker.getLastStartedRobotSide();
         // FIXME: I could see this going out of date and being wrong for multiple footsteps
         stanceSideWhenLastFootstepStarted = lastStartedRobotSide == null ? null : lastStartedRobotSide.getOppositeSide();
         behaviorState = behaviorStateReference.get();
         numberOfIncompleteFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps();
         swingPlannerType = SwingPlannerType.fromInt(lookAndStepParameters.getSwingPlannerType());

         planarRegionsManager.updateSnapshot();

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
   protected PlanarRegionsList lidarREAPlanarRegions;
   protected HeightMapData heightMapData;
   protected LookAndStepPlanarRegionsManager planarRegionsManager;
   protected CapturabilityBasedStatus capturabilityBasedStatus;
   protected RobotConfigurationData robotConfigurationData;
   protected TimerSnapshotWithExpiration planarRegionReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration heightMapReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration lidarREAPlanarRegionReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration capturabilityBasedStatusReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration robotConfigurationDataReceptionTimerSnapshot;
   protected TimerSnapshotWithExpiration planningFailureTimerSnapshot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected RobotSide stanceSideWhenLastFootstepStarted;
   protected RobotSide lastPlanInitialStanceSide;
   protected LookAndStepBehavior.State behaviorState;
   protected int numberOfIncompleteFootsteps;
   protected SwingPlannerType swingPlannerType;
   protected final List<FootstepStatusMessage> stepsStartedWhilePlanning = new ArrayList<>();
   protected final Stopwatch moreInclusivePlanningDurationStopwatch = new Stopwatch();
   private final Object logSessionSyncObject = new Object();

   protected void performTask()
   {
      moreInclusivePlanningDurationStopwatch.reset();
      // clear the list so we can inspect on completion
      stepsStartedWhilePlanning.clear();

      // detect flat ground; work in progress
      ConvexPolytope3D convexPolytope = new ConvexPolytope3D();
      convexPolytope.addVertices(Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getLeftFootSupportPolygon3d()));
      convexPolytope.addVertices(Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getRightFootSupportPolygon3d()));
      footholdVolume = convexPolytope.getVolume();

      SideDependentList<MinimalFootstep> startFootPoses = imminentStanceTracker.calculateImminentStancePoses();

      statusLogger.info(planarRegionsManager.computeRegionsToPlanWith(startFootPoses));

      PlanarRegionsList combinedRegionsForPlanning = new PlanarRegionsList();
      //      combinedRegionsForPlanning.addPlanarRegionsList(planarRegions);
      planarRegionsManager.getPlanarRegionsHistory().forEach(combinedRegionsForPlanning::addPlanarRegionsList);

      helper.publish(PLANAR_REGIONS_FOR_UI, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(combinedRegionsForPlanning));

      Pose3D closestPointAlongPath = localizationResult.getClosestPointAlongPath();
      int closestSegmentIndex = localizationResult.getClosestSegmentIndex();
      List<? extends Pose3DReadOnly> bodyPathPlan = localizationResult.getBodyPathPlan();

      // move point along body path plan by plan horizon
      Pose3D subGoalPoseBetweenFeet = new Pose3D();
      double planHorizonDistance = lookAndStepParameters.getPlanHorizon()
                                   + (lookAndStepParameters.getNumberOfStepsToTryToPlan() - 1) * footstepPlannerParameters.getMaximumStepReach();
      BodyPathPlannerTools.movePointAlongBodyPath(bodyPathPlan, closestPointAlongPath, subGoalPoseBetweenFeet, closestSegmentIndex, planHorizonDistance);

      statusLogger.info("Found next sub goal: {}", subGoalPoseBetweenFeet);

      // calculate impassibility
      if (lookAndStepParameters.getStopForImpassibilities() && lidarREAPlanarRegions != null)
      {
         Pose3D rootPose = new Pose3D(new Point3D(robotConfigurationData.getRootPosition()), robotConfigurationData.getRootOrientation());
         BodyCollisionData collisionData = PlannerTools.detectCollisionsAlongBodyPath(rootPose,
                                                                                      bodyPathPlan,
                                                                                      lidarREAPlanarRegions,
                                                                                      footstepPlannerParameters,
                                                                                      lookAndStepParameters.getHorizonFromDebrisToStop());
         if (collisionData != null && collisionData.isCollisionDetected())
         {
            Box3DMessage box3DMessage = new Box3DMessage();
            box3DMessage.getPose().set(collisionData.getBodyBox().getPose());
            box3DMessage.getSize().set(collisionData.getBodyBox().getSize());
            helper.publish(OBSTACLE, box3DMessage);
            helper.publish(IMPASSIBILITY_DETECTED, true);
            doFailureAction("Impassibility detected. Aborting task...");
            return;
         }
      }
      helper.publish(IMPASSIBILITY_DETECTED, false);

      // update last stepped poses to plan from; initialize to current poses
      MinimalFootstepListMessage imminentFootPosesForUI = new MinimalFootstepListMessage();
      for (RobotSide side : RobotSide.values)
      {
         MinimalFootstep startFootPose = startFootPoses.get(side);
         MinimalFootstepMessage minimalFootstepMessage = imminentFootPosesForUI.getMinimalFootsteps().add();
         minimalFootstepMessage.setRobotSide(startFootPose.getSide().toByte());
         minimalFootstepMessage.setDescription("Look and Step " + side.getPascalCaseName() + " Imminent");
         Pose3DReadOnly solePoseInWorld = startFootPose.getSolePoseInWorld();
         minimalFootstepMessage.getPosition().set(solePoseInWorld.getPosition());
         minimalFootstepMessage.getOrientation().set(solePoseInWorld.getOrientation());
         MinimalFootstep.packFootholdToMessage(startFootPose.getFoothold(), minimalFootstepMessage);
      }

      helper.publish(IMMINENT_FOOT_POSES_FOR_UI, imminentFootPosesForUI);

      RobotSide initialStanceSide;
      // if last plan failed
      // if foot is in the air
      // how many steps are left
      boolean isInMotion = RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus()) == RobotMotionStatus.IN_MOTION;
      if (isInMotion && stanceSideWhenLastFootstepStarted != null)
      {
         // If we are in motion (currently walking), make sure to plan w.r.t. the stance side when the last step started
         initialStanceSide = stanceSideWhenLastFootstepStarted.getOppositeSide();
      }
      // If we are stopped, prevent look and step from getting stuck if one side isn't feasible and alternate planning with left and right
      else if (lastPlanInitialStanceSide != null)
      {
         initialStanceSide = lastPlanInitialStanceSide.getOppositeSide();
      }
      else // if first step, step with furthest foot from the goal
      {
         if (startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld().getPosition().distance(subGoalPoseBetweenFeet.getPosition()) <= startFootPoses.get(
               RobotSide.RIGHT).getSolePoseInWorld().getPosition().distance(subGoalPoseBetweenFeet.getPosition()))
         {
            initialStanceSide = RobotSide.LEFT;
         }
         else
         {
            initialStanceSide = RobotSide.RIGHT;
         }
      }
      lastPlanInitialStanceSide = initialStanceSide;
      plannerFailedLastTime.set(false);

      helper.publish(SUB_GOAL_FOR_UI, subGoalPoseBetweenFeet);

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      //      footstepPlannerRequest.getBodyPathWaypoints().add(waypoint); // use these to add waypoints between start and goal
      footstepPlannerRequest.setRequestedInitialStanceSide(initialStanceSide);
      footstepPlannerRequest.setStartFootPoses(startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld(),
                                               startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld());

      // Make the timeout a little less so that we don't pause walking.
      double plannerTimeoutWhenMoving =  lookAndStepParameters.getPercentSwingToWait() * lookAndStepParameters.getSwingDuration() - 0.008;
      boolean robotIsInMotion = RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus()) == RobotMotionStatus.IN_MOTION;
      double plannerTimeout = robotIsInMotion ? plannerTimeoutWhenMoving : lookAndStepParameters.getFootstepPlannerTimeoutWhileStopped();
      // TODO: Set start footholds!!
      // TODO: only set square up steps at the end
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), subGoalPoseBetweenFeet);
      footstepPlannerRequest.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(combinedRegionsForPlanning)));
      footstepPlannerRequest.setTimeout(plannerTimeout);
      footstepPlannerRequest.setSwingPlannerType(swingPlannerType);
      footstepPlannerRequest.setSnapGoalSteps(true);
      footstepPlannerRequest.setMaximumIterations(100);

      double expirationTime = 1.5 * swingPlannerParameters.getMaximumSwingTime();
      if (successfulPlanExpirationTimer.isRunning(expirationTime)
          && previousFootstepPlan != null
          && previousFootstepPlan.getNumberOfSteps() >= 2)
      {
         if (initialStanceSide == previousFootstepPlan.getFootstep(0).getRobotSide())
         {
            previousFootstepPlan.remove(0);
         }
      }
      else
      {
         previousFootstepPlan = null;
      }
      footstepPlannerRequest.setReferencePlan(previousFootstepPlan);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.getSwingPlanningModule().getSwingPlannerParameters().set(swingPlannerParameters);
      footstepPlanningModule.clearCustomTerminationConditions();
      footstepPlanningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestSecondToFinalStep, bestPathSize) ->
                                                                 bestPathSize >= lookAndStepParameters.getNumberOfStepsToTryToPlan());
      MinimumFootstepChecker stepInPlaceChecker = new MinimumFootstepChecker();
      stepInPlaceChecker.setStanceFeetPoses(startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld(), startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld());
      footstepPlanningModule.getChecker().clearCustomFootstepCheckers();
      footstepPlanningModule.getChecker().attachCustomFootstepChecker(stepInPlaceChecker);
      int iterations = footstepPlanningModule.getAStarFootstepPlanner().getIterations();

      statusLogger.info("Stance side: {}", initialStanceSide.name());
      statusLogger.info("Planning footsteps with {}...", swingPlannerType.name());
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      statusLogger.info("Footstep planner completed with {}, {} step(s)",
                        footstepPlannerOutput.getFootstepPlanningResult(),
                        footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());
      statusLogger.info(StringTools.format3D("Planner timing took a total of {} s"
                                             + ", with {} s spent before planning and {} s spent planning"
                                             + ", and with a timeout of {} s",
                        footstepPlannerOutput.getPlannerTimings().getTotalElapsedSeconds(),
                        footstepPlannerOutput.getPlannerTimings().getTimeBeforePlanningSeconds(),
                        footstepPlannerOutput.getPlannerTimings().getTimePlanningStepsSeconds(),
                        plannerTimeout));
      footstepPlanningDuration = footstepPlannerOutput.getPlannerTimings().getTotalElapsedSeconds();

      String latestLogDirectory = FootstepPlannerLogger.generateALogFolderName();
      statusLogger.info("Footstep planner log folder: {}", latestLogDirectory);
      helper.publish(FOOTSTEP_PLANNER_LATEST_LOG_PATH, latestLogDirectory);
      ThreadTools.startAThread(() ->
      {
         synchronized (logSessionSyncObject)
         {
            Stopwatch stopwatch = new Stopwatch().start();
            FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
            footstepPlannerLogger.logSessionWithExactFolderName(latestLogDirectory);
            FootstepPlannerLogger.deleteOldLogs();
            LogTools.info("Logged footstep planner data in {} s. {} iterations",
                          FormattingTools.getFormattedDecimal3D(stopwatch.totalElapsed()),
                          iterations);
         }
      }, "FootstepPlanLogging");

      moreInclusivePlanningDuration = moreInclusivePlanningDurationStopwatch.lap();

      // TODO: Detect step down and reject unless we planned two steps.
      // Should get closer to the edge somehow?  Solve this in the footstep planner?
      if (footstepPlannerOutput.getFootstepPlan().isEmpty()) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanningModule);
         rejectionReasonReport.update();
         FootstepPlannerRejectionReasonsMessage footstepPlannerRejectionReasonsMessage = new FootstepPlannerRejectionReasonsMessage();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            FootstepPlannerRejectionReasonMessage footstepPlannerRejectionReasonMessage = footstepPlannerRejectionReasonsMessage.getRejectionReasons().add();
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            statusLogger.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
            footstepPlannerRejectionReasonMessage.setReason(reason.ordinal());
            footstepPlannerRejectionReasonMessage.setRejectionPercentage((float) MathTools.roundToSignificantFigures(rejectionPercentage, 3));
         }
         helper.publish(FOOTSTEP_PLANNER_REJECTION_REASONS, footstepPlannerRejectionReasonsMessage);
         helper.publish(PLANNING_FAILED, true);

         previousFootstepPlan = null;
         doFailureAction("Footstep planning failure. Aborting task...");
      }
      else
      {
         planarRegionsManager.dequeueToSize();

         FootstepPlan fullPlan = new FootstepPlan();
         for (int i = 0; i < footstepPlannerOutput.getFootstepPlan().getNumberOfSteps(); i++)
         {
            fullPlan.addFootstep(new PlannedFootstep(footstepPlannerOutput.getFootstepPlan().getFootstep(i)));
         }

         // This whole thing seems kinda dangerous
         if (stepsStartedWhilePlanning.size() > 0)
         {
            if (!removeStepsThatWereCompletedWhilePlanning(fullPlan))
               return;

            startFootPoses = imminentStanceTracker.calculateImminentStancePoses();
         }

         successfulPlanExpirationTimer.reset();
         previousFootstepPlan = new FootstepPlan(footstepPlannerOutput.getFootstepPlan());

//         if (!checkToMakeSurePlanIsStillReachable(fullPlan, startFootPoses))
//         {
//            uiPublisher.publishToUI(PlanningFailed, true);
//            doFailureAction("Footstep planning produced unreachable steps. Aborting task...");
//            return;
//         }

         FootstepPlan reducedPlan = new FootstepPlan();
         for (int i = 0; i < lookAndStepParameters.getMaxStepsToSendToController() && i < fullPlan.getNumberOfSteps(); i++)
         {
            reducedPlan.addFootstep(new PlannedFootstep(fullPlan.getFootstep(i)));
         }
         reducedPlan.setFinalTransferSplitFraction(fullPlan.getFinalTransferSplitFraction());
         reducedPlan.setFinalTransferWeightDistribution(fullPlan.getFinalTransferWeightDistribution());

         helper.publish(PLANNED_FOOTSTEPS_FOR_UI, MinimalFootstep.reduceFootstepPlanForUIROS2(reducedPlan, "Look and Step Planned"));

         updatePlannedFootstepDurations(reducedPlan, startFootPoses);

         if (operatorReviewEnabledSupplier.get())
         {
            if (lookAndStepParameters.getMaxStepsToSendToController() > 1)
               helper.getOrCreateRobotInterface().pauseWalking();
            review.review(reducedPlan);
         }
         else
         {
            autonomousOutput.accept(reducedPlan);
         }
      }
   }

   private boolean removeStepsThatWereCompletedWhilePlanning(FootstepPlan fullPlan)
   {
      for (int i = 0; i < stepsStartedWhilePlanning.size() && !fullPlan.isEmpty(); i++)
      {
         FootstepStatusMessage message = stepsStartedWhilePlanning.get(i);
         RobotSide swingSideStarted = RobotSide.fromByte(message.getRobotSide());

         if (fullPlan.getFootstep(0).getRobotSide() == swingSideStarted)
         {
            fullPlan.remove(0);
         }
         else
         {
            helper.publish(PLANNING_FAILED, true);
            doFailureAction("Footstep planning failure, our sequencing is wrong. Aborting task...");
            return false;
         }
      }
      if (fullPlan.isEmpty())
      {
         helper.publish(PLANNING_FAILED, true);
         doFailureAction("Footstep planning failure. We finished all the steps. Aborting task...");
         return false;
      }

      return true;
   }

   private boolean checkToMakeSurePlanIsStillReachable(FootstepPlan footstepPlan, SideDependentList<MinimalFootstep> startFootPoses)
   {
      PlannedFootstep firstStep = footstepPlan.getFootstep(0);
      FramePose3DReadOnly stepPose = firstStep.getFootstepPose();
      MinimalFootstep stanceFoot = startFootPoses.get(firstStep.getRobotSide().getOppositeSide());
      Pose3DReadOnly stancePose = stanceFoot.getSolePoseInWorld();

      if (stepPose.getPosition().distanceXY(stancePose.getPosition()) > footstepPlannerParameters.getMaximumStepReach())
         return false;

      return Math.abs(stepPose.getPosition().getZ() - stancePose.getZ()) < footstepPlannerParameters.getMaxStepZ();
   }


   private void updatePlannedFootstepDurations(FootstepPlan footstepPlan, SideDependentList<MinimalFootstep> startFootPoses)
   {
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
   }

   private void doFailureAction(String message)
   {
      // Finish the currently swinging step and stop walking
      helper.getOrCreateRobotInterface().pauseWalking();

      statusLogger.info(message);
      plannerFailedLastTime.set(true);
      planningFailedTimer.reset();
   }

   private class MinimumFootstepChecker implements CustomFootstepChecker
   {
      private final Pose3D leftFootStancePose = new Pose3D();
      private final Pose3D rightFootStancePose = new Pose3D();

      private final double minimumTranslation = lookAndStepParameters.getMinimumStepTranslation();
      private final double minimumRotation = Math.toRadians(lookAndStepParameters.getMinimumStepOrientation());

      public void setStanceFeetPoses(Pose3DReadOnly leftFootStancePose, Pose3DReadOnly rightFootStancePose)
      {
         this.leftFootStancePose.set(leftFootStancePose);
         this.rightFootStancePose.set(rightFootStancePose);
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
