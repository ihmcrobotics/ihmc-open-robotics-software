package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.Timer;
import us.ihmc.communication.util.TimerSnapshot;
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
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParametersReadOnly;
import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;
import us.ihmc.humanoidBehaviors.lookAndStep.TypedInput;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepFootstepPlanningPart
{
   protected StatusLogger statusLogger;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected FootstepPlannerParametersReadOnly footstepPlannerParameters;
   protected Supplier<Boolean> isBeingReviewedSupplier;
   protected Supplier<Boolean> abortGoalWalkingSupplier;
   protected UIPublisher uiPublisher;
   protected Runnable newBodyPathGoalNeededNotifier;
   protected Supplier<Boolean> newBodyPathGoalNeededSupplier;
   protected Function<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseSupplier;
   protected BiConsumer<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseConsumer;
   protected FootstepPlanningModule footstepPlanningModule;
   protected Consumer<RobotSide> lastStanceSideSetter;
   protected Supplier<Boolean> operatorReviewEnabledSupplier;
   protected Consumer<FootstepPlan> reviewPlanOutput;
   protected Consumer<FootstepPlan> autonomousOutput;
   protected Runnable planningFailedNotifier;
   protected Consumer<LookAndStepBehavior.State> behaviorStateUpdater;

   public static class TaskSetup extends LookAndStepFootstepPlanningPart
   {
      private final SingleThreadSizeOneQueueExecutor executor;
      private final Supplier<HumanoidRobotState> robotStateSupplier;
      private final Supplier<RobotSide> lastStanceSideSupplier;
      private final Supplier<LookAndStepBehavior.State> behaviorStateSupplier;

      private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
      private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
      private Timer planarRegionsExpirationTimer = new Timer();
      private Timer planningFailedTimer = new Timer();

      public TaskSetup(StatusLogger statusLogger,
                       LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                       FootstepPlannerParametersReadOnly footstepPlannerParameters,
                       Supplier<Boolean> abortGoalWalkingSupplier,
                       UIPublisher uiPublisher,
                       Runnable newBodyPathGoalNeededNotifier,
                       Supplier<Boolean> newBodyPathGoalNeededSupplier,
                       Function<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseSupplier, // TODO: Make these into one
                       BiConsumer<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseConsumer,
                       FootstepPlanningModule footstepPlanningModule,
                       Supplier<RobotSide> lastStanceSideSupplier,
                       Consumer<RobotSide> lastStanceSideSetter,
                       Supplier<Boolean> operatorReviewEnabledSupplier,
                       Supplier<HumanoidRobotState> robotStateSupplier,
                       Consumer<LookAndStepBehavior.State> behaviorStateUpdater,
                       Supplier<LookAndStepBehavior.State> behaviorStateSupplier)
      {
         this.statusLogger = statusLogger;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.footstepPlannerParameters = footstepPlannerParameters;
         this.abortGoalWalkingSupplier = abortGoalWalkingSupplier;
         this.uiPublisher = uiPublisher;
         this.newBodyPathGoalNeededNotifier = newBodyPathGoalNeededNotifier;
         this.newBodyPathGoalNeededSupplier = newBodyPathGoalNeededSupplier;
         this.lastSteppedSolePoseSupplier = lastSteppedSolePoseSupplier;
         this.lastSteppedSolePoseConsumer = lastSteppedSolePoseConsumer;
         this.footstepPlanningModule = footstepPlanningModule;
         this.lastStanceSideSetter = lastStanceSideSetter;
         this.operatorReviewEnabledSupplier = operatorReviewEnabledSupplier;
         this.behaviorStateUpdater = behaviorStateUpdater;
         this.robotStateSupplier = robotStateSupplier;
         this.lastStanceSideSupplier = lastStanceSideSupplier;
         this.behaviorStateSupplier = behaviorStateSupplier;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         planarRegionsInput.addCallback(data -> runOrQueue());
         bodyPathPlanInput.addCallback(data -> runOrQueue());

         this.planningFailedNotifier = planningFailedTimer::reset;
      }

      public void laterSetup(Supplier<Boolean> isBeingReviewedSupplier,
                             Consumer<FootstepPlan> reviewPlanOutput,
                             Consumer<FootstepPlan> autonomousOutput)
      {
         this.isBeingReviewedSupplier = isBeingReviewedSupplier;
         this.reviewPlanOutput = reviewPlanOutput;
         this.autonomousOutput = autonomousOutput;
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
         robotState = robotStateSupplier.get();
         lastStanceSide = lastStanceSideSupplier.get();
         behaviorState = behaviorStateSupplier.get();

         if (evaluateEntry())
         {
            performTask();
         }
      }
   }

   protected PlanarRegionsList planarRegions;
   protected TimerSnapshot planarRegionReceptionTimerSnapshot;
   protected TimerSnapshot planningFailureTimerSnapshot;
   protected List<? extends Pose3DReadOnly> bodyPathPlan;
   protected HumanoidRobotState robotState;
   protected RobotSide lastStanceSide;
   protected LookAndStepBehavior.State behaviorState;

   protected boolean evaluateEntry()
   {
      boolean proceed = true;

      if (!behaviorState.equals(LookAndStepBehavior.State.FOOTSTEP_PLANNING))
      {
         statusLogger.debug("Footstep planning supressed: Not in footstep planning state");
         statusLogger.debug("Footstep planning evauation failed: Not in footstep planning state");
         proceed = false;
      }
      else if (!regionsOK())
      {
         statusLogger.debug("Footstep planning suppressed: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                            planarRegions,
                            planarRegionReceptionTimerSnapshot.getTimePassedSinceReset(),
                            planarRegions == null ? null : planarRegions.isEmpty());

         proceed = false;
      }
      else if (planningFailureTimerSnapshot.isRunning())
      {
         statusLogger.debug("Footstep planning suppressed: Planning failed recently");
         proceed = false;
      }
      else if (!bodyPathPlanOK())
      {
         statusLogger.debug("Footstep planning suppressed: Body path size: {}", bodyPathPlan == null ? null : bodyPathPlan.size());
         proceed = false;
      }
      else if (isBeingReviewedSupplier.get())
      {
         statusLogger.debug("Footstep planning suppressed: Plan being reviewed");
         proceed = false;
      }
      else if (newBodyPathGoalNeededSupplier.get())
      {
         statusLogger.debug("Footstep planning suppressed: New body path goal needed");
         proceed = false;
      }

      return proceed;
   }

   private boolean regionsOK()
   {
      return planarRegions != null && !planarRegions.isEmpty() && planarRegionReceptionTimerSnapshot.isRunning();
   }

   private boolean bodyPathPlanOK()
   {
      return bodyPathPlan != null && !bodyPathPlan.isEmpty(); // are these null checks necessary?
   }

   protected void performTask()
   {
      statusLogger.info("Finding next sub goal for footstep planning...");
      uiPublisher.publishToUI(MapRegionsForUI, planarRegions);

      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(robotState.getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      FramePose3D pelvisPose = new FramePose3D();
      pelvisPose.setToZero(robotState.getPelvisFrame());
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
      if (reachedGoal)
      {
         statusLogger.warn("Footstep planning: Robot reached goal. Not planning");
         behaviorStateUpdater.accept(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         newBodyPathGoalNeededNotifier.run();
         return;
      }
      if (abortGoalWalkingSupplier.get())
      {
         statusLogger.warn("Footstep planning: Goal was cancelled. Not planning");
         behaviorStateUpdater.accept(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         newBodyPathGoalNeededNotifier.run();
         return;
      }

      //      double trailingBy = goalPoseBetweenFeet.getPositionDistance(initialPoseBetweenFeet);
      //      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(lookAndStepParameters.get(LookAndStepBehaviorParameters.direction), 0.0, 0.0);
      //      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.planHorizon) - trailingBy, 0.0, 0.0);

      //      Vector2D headingVector = new Vector2D();
      //      headingVector.set(goalPoint.getX(), goalPoint.getY());
      //      headingVector.sub(goalPoseBetweenFeet.getPosition().getX(), goalPoseBetweenFeet.getPosition().getY());

      statusLogger.info("Found next sub goal: {}", goalPoint);
      goalPoseBetweenFeet.getPosition().set(goalPoint);

      //      double yaw = Math.atan2(headingVector.getX(), headingVector.getY());
      //      LogTools.info("Setting yaw: {}", yaw);
      //      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);

      goalPoseBetweenFeet.getOrientation().set(bodyPathPlan.get(segmentIndexOfGoal + 1).getOrientation());

      // update last stepped poses to plan from; initialize to current poses

      SideDependentList<FramePose3DReadOnly> startFootPoses = new SideDependentList<>();
      ArrayList<FootstepForUI> startFootPosesForUI = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         FramePose3DReadOnly lastSteppedSolePose = lastSteppedSolePoseSupplier.apply(side);
         if (lastSteppedSolePose == null)
         {
            FramePose3D soleFrameZUpPose = new FramePose3D(robotState.getSoleZUpFrame(side));
            soleFrameZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
            lastSteppedSolePose = soleFrameZUpPose;
            lastSteppedSolePoseConsumer.accept(side, lastSteppedSolePose);
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

      lastStanceSideSetter.accept(stanceSide);

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
      //      footstepPlanningModule.addStatusCallback(this::footstepPlanningStatusUpdate);
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
         reviewPlanOutput.accept(footstepPlannerOutput.getFootstepPlan());
      }
      else
      {
         behaviorStateUpdater.accept(LookAndStepBehavior.State.SWINGING);
         autonomousOutput.accept(footstepPlannerOutput.getFootstepPlan());
      }
   }
}
