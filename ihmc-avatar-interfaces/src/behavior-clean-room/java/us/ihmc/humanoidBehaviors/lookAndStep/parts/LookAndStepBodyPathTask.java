package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.util.TimerSnapshot;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParametersReadOnly;
import us.ihmc.humanoidBehaviors.tools.BehaviorBuilderPattern;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.MapRegionsForUI;

public class LookAndStepBodyPathTask implements BehaviorBuilderPattern
{
   protected final Field<UIPublisher> uiPublisher = required();
   protected final Field<VisibilityGraphsParametersReadOnly> visibilityGraphParameters = required();
   protected final Field<LookAndStepBehaviorParametersReadOnly> lookAndStepBehaviorParameters = required();
   protected final Field<Supplier<Boolean>> operatorReviewEnabled = required();
   protected final Field<Consumer<ArrayList<Pose3D>>> successfulPlanConsumer = required();
   protected final Field<Runnable> resetPlanningFailedTimer = required();
   protected final Field<Consumer<List<? extends Pose3DReadOnly>>> initiateReviewOutput = required();
   protected final Field<Supplier<Boolean>> isBeingReviewed = required();
   protected final Field<Supplier<Boolean>> needNewPlan = required();
   protected final Field<Runnable> clearNewBodyPathNeededCallback = required();
   protected final Field<Consumer<LookAndStepBehavior.State>> behaviorStateUpdater = required();

   private PlanarRegionsList mapRegions;
   private Pose3D goal;
   private HumanoidRobotState humanoidRobotState;
   private TimerSnapshot mapRegionsReceptionTimerSnapshot;
   private TimerSnapshot planningFailureTimerSnapshot;
   private LookAndStepBehavior.State behaviorState;

   protected void update(PlanarRegionsList mapRegions,
                         Pose3D goal,
                         HumanoidRobotState humanoidRobotState,
                         TimerSnapshot mapRegionsReceptionTimerSnapshot,
                         TimerSnapshot planningFailureTimerSnapshot,
                         LookAndStepBehavior.State behaviorState)
   {
      this.mapRegions = mapRegions;
      this.goal = goal;
      this.humanoidRobotState = humanoidRobotState;
      this.mapRegionsReceptionTimerSnapshot = mapRegionsReceptionTimerSnapshot;
      this.planningFailureTimerSnapshot = planningFailureTimerSnapshot;
      this.behaviorState = behaviorState;
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
         LogTools.warn("Body path planning supressed: Not in body path planning state");
         proceed = false;
      }
      else if (!hasGoal())
      {
         LogTools.warn("Body path planning supressed: No goal specified");
         uiPublisher.get().publishToUI(MapRegionsForUI, mapRegions);
         proceed = false;
      }
      else if (!regionsOK())
      {
         LogTools.warn("Body path planning supressed: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                       mapRegions,
                       mapRegionsReceptionTimerSnapshot.getTimePassedSinceReset(),
                       mapRegions == null ? null : mapRegions.isEmpty());
         proceed = false;
      }
      else if (planningFailureTimerSnapshot.isRunning()) // TODO: This could be "run recently" instead of failed recently
      {
         LogTools.warn("Body path planning supressed: Failed recently");
         proceed = false;
      }
      else if (isBeingReviewed.get().get())
      {
         LogTools.warn("Body path planning supressed: Is being reviewed");
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
      return mapRegions != null
             && !mapRegions.isEmpty()
             && mapRegionsReceptionTimerSnapshot.isRunning();
   }

   // TODO: Extract as interface?
   public void run()
   {
      validateAll();

      if (evaluateEntry())
      {
         performTask();
      }

      invalidateChanging();
   }

   private void performTask()
   {
      // TODO: Add robot standing still for 20s for real robot?
      uiPublisher.get().publishToUI(MapRegionsForUI, mapRegions);

      LogTools.info("Planning body path...");

      clearNewBodyPathNeededCallback.get().run();

      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters.get());
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters.get(),
                                                                                  pathPostProcessor,
                                                                                  new YoVariableRegistry(LookAndStepBodyPathModule.class.getSimpleName()));

      bodyPathPlanner.setGoal(goal);
      bodyPathPlanner.setPlanarRegionsList(mapRegions);
      FramePose3D leftFootPoseTemp = new FramePose3D();
      leftFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPoseTemp = new FramePose3D();
      rightFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPathPlanner.setStanceFootPoses(leftFootPoseTemp, rightFootPoseTemp);
      Stopwatch stopwatch = new Stopwatch().start();
      final ArrayList<Pose3D> bodyPathPlanForReview = new ArrayList<>(); // TODO Review making this final
      bodyPathPlanner.planWaypoints(); // takes about 0.1s
      LogTools.info("Body path planning took {}, contains {} waypoint", stopwatch.totalElapsed(), bodyPathPlanForReview.size());
      //      bodyPathPlan = bodyPathPlanner.getWaypoints();
      if (bodyPathPlanner.getWaypoints() != null)
      {
         for (Pose3DReadOnly poseWaypoint : bodyPathPlanner.getWaypoints())
         {
            bodyPathPlanForReview.add(new Pose3D(poseWaypoint));
         }
         uiPublisher.get().publishToUI(BodyPathPlanForUI, bodyPathPlanForReview);
      }

      if (bodyPathPlanForReview.size() >= 2)
      {
         if (operatorReviewEnabled.get().get())
         {
            initiateReviewOutput.get().accept(bodyPathPlanForReview);
         }
         else
         {
            behaviorStateUpdater.get().accept(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
            successfulPlanConsumer.get().accept(bodyPathPlanForReview);
         }
      }
      else
      {
         resetPlanningFailedTimer.get().run();
      }
   }

   public void setUIPublisher(UIPublisher uiPublisher)
   {
      this.uiPublisher.set(uiPublisher);
   }

   public void setVisibilityGraphParameters(VisibilityGraphsParametersReadOnly visibilityGraphParameters)
   {
      this.visibilityGraphParameters.set(visibilityGraphParameters);
   }

   public void setLookAndStepBehaviorParameters(LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters)
   {
      this.lookAndStepBehaviorParameters.set(lookAndStepBehaviorParameters);
   }

   public void setOperatorReviewEnabled(Supplier<Boolean> operatorReviewEnabled)
   {
      this.operatorReviewEnabled.set(operatorReviewEnabled);
   }

   public void setAutonomousOutput(Consumer<ArrayList<Pose3D>> successfulPlanConsumer)
   {
      this.successfulPlanConsumer.set(successfulPlanConsumer);
   }

   public void setResetPlanningFailedTimer(Runnable resetPlanningFailedTimer)
   {
      this.resetPlanningFailedTimer.set(resetPlanningFailedTimer);
   }

   public void setReviewInitiator(Consumer<List<? extends Pose3DReadOnly>> reviewInitiation)
   {
      this.initiateReviewOutput.set(reviewInitiation);
   }

   public void setIsBeingReviewedSupplier(Supplier<Boolean> isBeingReviewed)
   {
      this.isBeingReviewed.set(isBeingReviewed);
   }

   public void setNeedNewPlan(Supplier<Boolean> needNewPlan)
   {
      this.needNewPlan.set(needNewPlan);
   }

   public void setClearNewBodyPathGoalNeededCallback(Runnable clearNewBodyPathNeededCallback)
   {
      this.clearNewBodyPathNeededCallback.set(clearNewBodyPathNeededCallback);
   }

   public void setBehaviorStateUpdater(Consumer<LookAndStepBehavior.State> behaviorStateUpdater)
   {
      this.behaviorStateUpdater.set(behaviorStateUpdater);
   }
}
