package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.commons.time.Stopwatch;
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
import us.ihmc.humanoidBehaviors.tools.BehaviorBuilderPattern;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.log.LogTools;
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

public class LookAndStepFootstepPlanningTask implements BehaviorBuilderPattern
{
   protected final Field<LookAndStepBehaviorParametersReadOnly> lookAndStepBehaviorParameters = required(); // TODO: Maybe we only need to invalidate some fields
   protected final Field<FootstepPlannerParametersReadOnly> footstepPlannerParameters = required();
   protected final Field<Supplier<Boolean>> isBeingReviewedSupplier = required();
   protected final Field<Supplier<Boolean>> abortGoalWalkingSupplier = required();
   protected final Field<Consumer<Boolean>> abortGoalWalkingUpdater = required();
   protected final Field<UIPublisher> uiPublisher = required();
   protected final Field<Runnable> newBodyPathGoalNeededNotifier = required();
   protected final Field<Supplier<Boolean>> newBodyPathGoalNeededSupplier = required();
   protected final Field<Function<RobotSide, FramePose3DReadOnly>> lastSteppedSolePoseSupplier = required();
   protected final Field<BiConsumer<RobotSide, FramePose3DReadOnly>> lastSteppedSolePoseConsumer = required();
   protected final Field<FootstepPlanningModule> footstepPlanningModule = required();
   protected final Field<Consumer<RobotSide>> lastStanceSideSetter = required();
   protected final Field<Supplier<Boolean>> operatorReviewEnabledSupplier = required();
   protected final Field<Consumer<FootstepPlan>> reviewPlanOutput = required();
   protected final Field<Consumer<FootstepPlan>> autonomousOutput = required();
   protected final Field<Runnable> planningFailedNotifier = required();
   protected final Field<Consumer<LookAndStepBehavior.State>> behaviorStateUpdater = required();
   protected final Field<Consumer<String>> statusLogger = required();

   private PlanarRegionsList planarRegions;
   private TimerSnapshot planarRegionReceptionTimerSnapshot;
   private TimerSnapshot planningFailureTimerSnapshot;
   private List<? extends Pose3DReadOnly> bodyPathPlan;
   private HumanoidRobotState robotState;
   private RobotSide lastStanceSide;
   private LookAndStepBehavior.State behaviorState;

   protected void update(PlanarRegionsList planarRegions,
                         TimerSnapshot planarRegionReceptionTimerSnapshot,
                         TimerSnapshot planningFailureTimerSnapshot,
                         List<? extends Pose3DReadOnly> bodyPathPlan,
                         HumanoidRobotState robotState,
                         RobotSide lastStanceSide,
                         LookAndStepBehavior.State behaviorState)
   {
      this.planarRegions = planarRegions;
      this.planarRegionReceptionTimerSnapshot = planarRegionReceptionTimerSnapshot;
      this.planningFailureTimerSnapshot = planningFailureTimerSnapshot;
      this.bodyPathPlan = bodyPathPlan;
      this.robotState = robotState;
      this.lastStanceSide = lastStanceSide;
      this.behaviorState = behaviorState;
   }

   private boolean evaluateEntry()
   {
      boolean proceed = true;

      if (!behaviorState.equals(LookAndStepBehavior.State.FOOTSTEP_PLANNING))
      {
         LogTools.warn("Footstep planning supressed: Not in footstep planning state");
         statusLogger.get().accept("Footstep planning evauation failed: Not in footstep planning state");
         proceed = false;
      }
      else if (!regionsOK())
      {
         LogTools.warn("Footstep planning suppressed: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                       planarRegions,
                       planarRegionReceptionTimerSnapshot.getTimePassedSinceReset(),
                       planarRegions == null ? null : planarRegions.isEmpty());

         proceed = false;
      }
      else if (planningFailureTimerSnapshot.isRunning())
      {
         LogTools.warn("Footstep planning suppressed: Planning failed recently");
         proceed = false;
      }
      else if (!bodyPathPlanOK())
      {
         LogTools.warn("Footstep planning suppressed: Body path size: {}", bodyPathPlan == null ? null : bodyPathPlan.size());
         proceed = false;
      }
      else if (isBeingReviewedSupplier.get().get())
      {
         LogTools.warn("Footstep planning suppressed: Plan being reviewed");
         proceed = false;
      }
      else if (newBodyPathGoalNeededSupplier.get().get())
      {
         LogTools.warn("Footstep planning suppressed: New body path goal needed");
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

   private void performTask()
   {
      LogTools.info("Finding next sub goal for footstep planning...");
      uiPublisher.get().publishToUI(MapRegionsForUI, planarRegions);

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

      uiPublisher.get().publishToUI(ClosestPointForUI, new Pose3D(closestPointAlongPath, new Quaternion()));

      // move point along body path plan by plan horizon
      Point3D goalPoint = new Point3D();
      int segmentIndexOfGoal = BodyPathPlannerTools.movePointAlongBodyPath(bodyPathPlan,
                                                                           closestPointAlongPath,
                                                                           goalPoint,
                                                                           closestSegmentIndex,
                                                                           lookAndStepBehaviorParameters.get().getPlanHorizon());

      Pose3DReadOnly terminalGoal = bodyPathPlan.get(bodyPathPlan.size() - 1);
      boolean reachedGoal = closestPointAlongPath.distanceXY(goalPoint) < lookAndStepBehaviorParameters.get().getGoalSatisfactionRadius();
      reachedGoal &= Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pelvisPose.getYaw(), terminalGoal.getYaw())) < lookAndStepBehaviorParameters.get().getGoalSatisfactionOrientationDelta();
      if (reachedGoal)
      {
         LogTools.warn("Footstep planning: Robot reached goal. Not planning");
         behaviorStateUpdater.get().accept(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         newBodyPathGoalNeededNotifier.get().run();
         return;
      }
      if (abortGoalWalkingSupplier.get().get())
      {
         LogTools.warn("Footstep planning: Goal was cancelled. Not planning");
         behaviorStateUpdater.get().accept(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         newBodyPathGoalNeededNotifier.get().run();
         abortGoalWalkingUpdater.get().accept(false);
         return;
      }

      //      double trailingBy = goalPoseBetweenFeet.getPositionDistance(initialPoseBetweenFeet);
      //      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(lookAndStepParameters.get(LookAndStepBehaviorParameters.direction), 0.0, 0.0);
      //      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.planHorizon) - trailingBy, 0.0, 0.0);

      //      Vector2D headingVector = new Vector2D();
      //      headingVector.set(goalPoint.getX(), goalPoint.getY());
      //      headingVector.sub(goalPoseBetweenFeet.getPosition().getX(), goalPoseBetweenFeet.getPosition().getY());

      LogTools.info("Setting goalPoint: {}", goalPoint);
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
         FramePose3DReadOnly lastSteppedSolePose = lastSteppedSolePoseSupplier.get().apply(side);
         if (lastSteppedSolePose == null)
         {
            FramePose3D soleFrameZUpPose = new FramePose3D(robotState.getSoleZUpFrame(side));
            soleFrameZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
            lastSteppedSolePose = soleFrameZUpPose;
            lastSteppedSolePoseConsumer.get().accept(side, lastSteppedSolePose);
         }

         startFootPosesForUI.add(new FootstepForUI(side, new Pose3D(lastSteppedSolePose), side.getPascalCaseName() + " Start"));
         startFootPoses.put(side, lastSteppedSolePose);
      }
      uiPublisher.get().publishToUI(StartAndGoalFootPosesForUI, startFootPosesForUI);

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

      lastStanceSideSetter.get().accept(stanceSide);

      uiPublisher.get().publishToUI(SubGoalForUI, new Pose3D(goalPoseBetweenFeet));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(startFootPoses.get(RobotSide.LEFT), startFootPoses.get(RobotSide.RIGHT));
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.get().getIdealFootstepWidth(), goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(planarRegions);
      footstepPlannerRequest.setTimeout(lookAndStepBehaviorParameters.get().getFootstepPlannerTimeout());
      footstepPlannerRequest.setSwingPlannerType(SwingPlannerType.POSITION);

      footstepPlanningModule.get().getFootstepPlannerParameters().set(footstepPlannerParameters.get());
//      footstepPlanningModule.get().addStatusCallback(this::footstepPlanningStatusUpdate);
      footstepPlanningModule.get().addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestPathSize) -> bestPathSize >= lookAndStepBehaviorParameters.get().getMinimumNumberOfPlannedSteps());

      LogTools.info("Footstep planner started...");
      Stopwatch stopwatch = new Stopwatch().start();
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.get().handleRequest(footstepPlannerRequest);

      LogTools.info("Footstep planner completed in {} s!", stopwatch.totalElapsed());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule.get());
      footstepPlannerLogger.logSession();
      FootstepPlannerLogger.deleteOldLogs(30); // TODO: Do this somewhere else

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         planningFailedNotifier.get().run();
      }
      else
      {
         uiPublisher.get().publishToUI(FootstepPlanForUI, FootstepForUI.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(), "Planned"));
      }

      if (operatorReviewEnabledSupplier.get().get())
      {
         reviewPlanOutput.get().accept(footstepPlannerOutput.getFootstepPlan());
      }
      else
      {
         behaviorStateUpdater.get().accept(LookAndStepBehavior.State.SWINGING);
         autonomousOutput.get().accept(footstepPlannerOutput.getFootstepPlan());
      }
   }

   public void run()
   {
      validateAll();

      if (evaluateEntry())
      {
         performTask();
      }

      invalidateChanging();
   }

   public void setIsBeingReviewedSupplier(Supplier<Boolean> isBeingReviewedSupplier)
   {
      this.isBeingReviewedSupplier.set(isBeingReviewedSupplier);
   }

   public void setAbortGoalWalkingSupplier(Supplier<Boolean> abortGoalWalkingSupplier)
   {
      this.abortGoalWalkingSupplier.set(abortGoalWalkingSupplier);
   }

   public void setAbortGoalWalkingUpdater(Consumer<Boolean> abortGoalWalkingUpdater)
   {
      this.abortGoalWalkingUpdater.set(abortGoalWalkingUpdater);
   }

   public void setUiPublisher(UIPublisher uiPublisher)
   {
      this.uiPublisher.set(uiPublisher);
   }

   public void setLookAndStepBehaviorParameters(LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters)
   {
      this.lookAndStepBehaviorParameters.set(lookAndStepBehaviorParameters);
   }

   public void setFootstepPlannerParameters(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.footstepPlannerParameters.set(footstepPlannerParameters);
   }

   public void setNewBodyPathGoalNeededNotifier(Runnable newBodyPathGoalNeededNotifier)
   {
      this.newBodyPathGoalNeededNotifier.set(newBodyPathGoalNeededNotifier);
   }

   public void setNewBodyPathGoalNeededSupplier(Supplier<Boolean> newBodyPathGoalNeededSupplier)
   {
      this.newBodyPathGoalNeededSupplier.set(newBodyPathGoalNeededSupplier);
   }

   public void setLastSteppedSolePoseSupplier(Function<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseSupplier)
   {
      this.lastSteppedSolePoseSupplier.set(lastSteppedSolePoseSupplier);
   }

   public void setLastSteppedSolePoseConsumer(BiConsumer<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseConsumer)
   {
      this.lastSteppedSolePoseConsumer.set(lastSteppedSolePoseConsumer);
   }

   public void setLastStanceSideSetter(Consumer<RobotSide> lastStanceSideSetter)
   {
      this.lastStanceSideSetter.set(lastStanceSideSetter);
   }

   public void setFootstepPlanningModule(FootstepPlanningModule footstepPlanningModule)
   {
      this.footstepPlanningModule.set(footstepPlanningModule);
   }

   public void setOperatorReviewEnabledSupplier(Supplier<Boolean> operatorReviewEnabledSupplier)
   {
      this.operatorReviewEnabledSupplier.set(operatorReviewEnabledSupplier);
   }

   public void setReviewPlanOutput(Consumer<FootstepPlan> reviewPlanOutput)
   {
      this.reviewPlanOutput.set(reviewPlanOutput);
   }

   public void setAutonomousOutput(Consumer<FootstepPlan> autonomousOutput)
   {
      this.autonomousOutput.set(autonomousOutput);
   }

   protected void setPlanningFailedNotifier(Runnable planningFailedNotifier)
   {
      this.planningFailedNotifier.set(planningFailedNotifier);
   }

   public void setBehaviorStateUpdater(Consumer<LookAndStepBehavior.State> behaviorStateUpdater)
   {
      this.behaviorStateUpdater.set(behaviorStateUpdater);
   }

   public void setStatusLogger(Consumer<String> statusLogger)
   {
      this.statusLogger.set(statusLogger);
   }
}
