package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.communication.util.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.lookAndStep.BehaviorStateReference;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParametersReadOnly;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.MapRegionsForUI;

class LookAndStepBodyPathTask
{
   protected final StatusLogger statusLogger;
   protected final UIPublisher uiPublisher;
   protected final VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   protected final LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected final Supplier<Boolean> operatorReviewEnabled;
   protected final BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference;
   protected final Supplier<Boolean> robotConnectedSupplier;

   protected Consumer<ArrayList<Pose3D>> autonomousOutput;
   protected Consumer<List<? extends Pose3DReadOnly>> initiateReviewOutput;
   protected Supplier<Boolean> isBeingReviewed;

   protected Runnable resetPlanningFailedTimer;

   private PlanarRegionsList mapRegions;
   private Pose3D goal;
   private RemoteSyncedRobotModel syncedRobot;
   private TimerSnapshotWithExpiration mapRegionsReceptionTimerSnapshot;
   private TimerSnapshotWithExpiration planningFailureTimerSnapshot;
   private LookAndStepBehavior.State behaviorState;
   private int numberOfIncompleteFootsteps;

   LookAndStepBodyPathTask(StatusLogger statusLogger,
                           UIPublisher uiPublisher,
                           VisibilityGraphsParametersReadOnly visibilityGraphParameters,
                           LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                           Supplier<Boolean> operatorReviewEnabled,
                           BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference,
                           Supplier<Boolean> robotConnectedSupplier)
   {
      this.statusLogger = statusLogger;
      this.uiPublisher = uiPublisher;
      this.visibilityGraphParameters = visibilityGraphParameters;
      this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
      this.operatorReviewEnabled = operatorReviewEnabled;
      this.behaviorStateReference = behaviorStateReference;
      this.robotConnectedSupplier = robotConnectedSupplier;
   }

   protected void update(PlanarRegionsList mapRegions,
                         Pose3D goal,
                         RemoteSyncedRobotModel humanoidRobotState,
                         TimerSnapshotWithExpiration mapRegionsReceptionTimerSnapshot,
                         TimerSnapshotWithExpiration planningFailureTimerSnapshot,
                         LookAndStepBehavior.State behaviorState,
                         int numberOfIncompleteFootsteps)
   {
      this.mapRegions = mapRegions;
      this.goal = goal;
      this.syncedRobot = humanoidRobotState;
      this.mapRegionsReceptionTimerSnapshot = mapRegionsReceptionTimerSnapshot;
      this.planningFailureTimerSnapshot = planningFailureTimerSnapshot;
      this.behaviorState = behaviorState;
      this.numberOfIncompleteFootsteps = numberOfIncompleteFootsteps;
   }

   private boolean evaluateEntry()
   {
      boolean proceed = true;

      //      if (!needNewPlan.get().get())
      //      {
      //         LogTools.warn("Body path planning supressed: New plan not needed");
      //         proceed = false;
      //      }
      if (!behaviorState.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING))
      {
         statusLogger.debug("Body path planning suppressed: Not in body path planning state");
         proceed = false;
      }
      else if (!hasGoal())
      {
         statusLogger.debug("Body path planning suppressed: No goal specified");
         uiPublisher.publishToUI(MapRegionsForUI, mapRegions);
         proceed = false;
      }
      else if (!regionsOK())
      {
         statusLogger.debug("Body path planning suppressed: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                            mapRegions,
                            mapRegionsReceptionTimerSnapshot.getTimePassedSinceReset(),
                            mapRegions == null ? null : mapRegions.isEmpty());
         proceed = false;
      }
      else if (planningFailureTimerSnapshot.isRunning()) // TODO: This could be "run recently" instead of failed recently
      {
         statusLogger.debug("Body path planning suppressed: Failed recently");
         proceed = false;
      }
      else if (isBeingReviewed.get())
      {
         statusLogger.debug("Body path planning suppressed: Is being reviewed");
         proceed = false;
      }
      else if (!robotConnectedSupplier.get())
      {
         statusLogger.debug("Body path planning suppressed: Robot disconnected");
         proceed = false;
      }
      else if (numberOfIncompleteFootsteps > lookAndStepBehaviorParameters.getAcceptableIncompleteFootsteps())
      {
         statusLogger.debug("Body path planning suppressed: numberOfIncompleteFootsteps {} > {}",
                            numberOfIncompleteFootsteps,
                            lookAndStepBehaviorParameters.getAcceptableIncompleteFootsteps());
         proceed = false;
      }

      return proceed;
   }

   private boolean hasGoal()
   {
      return goal != null && !goal.containsNaN();
   }

   private boolean regionsOK()
   {
      return mapRegions != null && !mapRegions.isEmpty() && mapRegionsReceptionTimerSnapshot.isRunning();
   }

   // TODO: Extract as interface?
   public void run()
   {
      Objects.requireNonNull(resetPlanningFailedTimer);
      Objects.requireNonNull(isBeingReviewed);
      Objects.requireNonNull(autonomousOutput);
      Objects.requireNonNull(initiateReviewOutput);

      if (evaluateEntry())
      {
         performTask();
      }
   }

   private void performTask()
   {
      statusLogger.info("Body path planning...");
      // TODO: Add robot standing still for 20s for real robot?
      uiPublisher.publishToUI(MapRegionsForUI, mapRegions);

      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      YoVariableRegistry parentRegistry = new YoVariableRegistry(LookAndStepBodyPathModule.class.getSimpleName());
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
      bodyPathPlanner.planWaypoints(); // takes about 0.1s
      statusLogger.info("Body path plan completed with {} waypoint(s)", bodyPathPlanner.getWaypoints().size());
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
            initiateReviewOutput.accept(bodyPathPlanForReview);
         }
         else
         {
            behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
            autonomousOutput.accept(bodyPathPlanForReview);
         }
      }
      else
      {
         resetPlanningFailedTimer.run();
      }
   }

   protected void setResetPlanningFailedTimer(Runnable resetPlanningFailedTimer)
   {
      this.resetPlanningFailedTimer = resetPlanningFailedTimer;
   }

   public void setIsBeingReviewedSupplier(Supplier<Boolean> isBeingReviewed)
   {
      this.isBeingReviewed = isBeingReviewed;
   }

   public void setAutonomousOutput(Consumer<ArrayList<Pose3D>> autonomousOutput)
   {
      this.autonomousOutput = autonomousOutput;
   }

   public void setReviewInitiator(Consumer<List<? extends Pose3DReadOnly>> reviewInitiation)
   {
      this.initiateReviewOutput = reviewInitiation;
   }
}
