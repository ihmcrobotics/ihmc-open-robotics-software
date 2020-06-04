package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
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

import java.time.LocalDateTime;
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

   private final Field<PlanarRegionsList> mapRegions = requiredChanging();
   private final Field<Pose3D> goal = requiredChanging();
   private final Field<HumanoidRobotState> humanoidRobotState = requiredChanging();
   private final Field<SimpleTimer.Status> mapRegionsExpirationStatus = requiredChanging();
   private final Field<SimpleTimer.Status> planningFailedTimerStatus = requiredChanging();

   private boolean evaluateEntry()
   {
      boolean proceed = true;

      if (!hasGoal())
      {
         LogTools.warn("Body path: does not have goal");
         LogTools.debug("Sending planar regions to UI: {}: {}", LocalDateTime.now(), mapRegions.hashCode());
         uiPublisher.get().publishToUI(MapRegionsForUI, mapRegions.get());
         proceed = false;
      }
      else if (!regionsOK())
      {
         LogTools.warn("Body path: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                       mapRegions,
                       mapRegionsExpirationStatus.get().getTimePassedSinceReset(),
                       mapRegions == null ? null : mapRegions.get().isEmpty());
         proceed = false;
      }
      else if (!needNewPlan.get().get())
      {
         LogTools.warn("Body path: New plan not needed");
         proceed = false;
      }
      else if (failedRecently()) // TODO: This could be "run recently" instead of failed recently
      {
         LogTools.warn("Body path: failedRecently = true");
         proceed = false;
      }
      else if (isBeingReviewed.get().get())
      {
         LogTools.debug("Body path: bodyPathBeingReviewed = true");
         proceed = false;
      }

      return proceed;
   }

   private boolean hasGoal()
   {
      return goal.get() != null && !goal.get().containsNaN();
   }

   private boolean regionsOK()
   {
      return mapRegions.get() != null && !mapRegionsExpirationStatus.get().isPastOrNaN() && !mapRegions.get().isEmpty();
   }

   private boolean failedRecently()
   {
      return !planningFailedTimerStatus.get().isPastOrNaN();
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
      uiPublisher.get().publishToUI(MapRegionsForUI, mapRegions.get());

      LogTools.info("Planning body path...");

      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters.get());
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters.get(),
                                                                                  pathPostProcessor,
                                                                                  new YoVariableRegistry(LookAndStepBodyPathModule.class.getSimpleName()));

      bodyPathPlanner.setGoal(goal.get());
      bodyPathPlanner.setPlanarRegionsList(mapRegions.get());
      FramePose3D leftFootPoseTemp = new FramePose3D();
      leftFootPoseTemp.setToZero(humanoidRobotState.get().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPoseTemp = new FramePose3D();
      rightFootPoseTemp.setToZero(humanoidRobotState.get().getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPathPlanner.setStanceFootPoses(leftFootPoseTemp, rightFootPoseTemp);
      Stopwatch stopwatch = new Stopwatch().start();
      final ArrayList<Pose3D> bodyPathPlanForReview = new ArrayList<>(); // TODO Review making this final
      bodyPathPlanner.planWaypoints();
      LogTools.info("Body path planning took {}", stopwatch.totalElapsed()); // 0.1 s
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
            successfulPlanConsumer.get().accept(bodyPathPlanForReview);
         }
      }
      else
      {
         resetPlanningFailedTimer.get().run();
      }
   }

   protected void setMapRegions(PlanarRegionsList mapRegions)
   {
      this.mapRegions.set(mapRegions);
   }

   protected void setGoal(Pose3D goal)
   {
      this.goal.set(goal);
   }

   protected void setHumanoidRobotState(HumanoidRobotState humanoidRobotState)
   {
      this.humanoidRobotState.set(humanoidRobotState);
   }

   protected void setMapRegionsExpirationStatus(SimpleTimer.Status mapRegionsExpirationStatus)
   {
      this.mapRegionsExpirationStatus.set(mapRegionsExpirationStatus);
   }

   protected void setPlanningFailedTimerStatus(SimpleTimer.Status planningFailedTimerStatus)
   {
      this.planningFailedTimerStatus.set(planningFailedTimerStatus);
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
}
