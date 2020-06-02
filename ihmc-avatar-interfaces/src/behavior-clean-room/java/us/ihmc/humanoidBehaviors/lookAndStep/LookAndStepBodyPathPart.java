package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.log.LogTools;
import us.ihmc.messager.TopicListener;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.MapRegionsForUI;

public class LookAndStepBodyPathPart
{
   private final BehaviorHelper helper;
   private final Supplier<Boolean> isBeingReviewed;
   private final Supplier<Boolean> operatorReviewEnabled;
   private final Supplier<HumanoidRobotState> robotStateSupplier;
   private final LookAndStepBehaviorParameters lookAndStepParameters;
   private final VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   private final Consumer<ArrayList<Pose3D>> successfulPlanConsumer;
   private final Consumer<ArrayList<Pose3D>> review;
   private final Supplier<Boolean> needNewPlan;

   private final TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
   private SimpleTimer mapRegionsExpirationTimer = new SimpleTimer();

   private final TypedInput<Pose3D> goalInput = new TypedInput<>();

   private SimpleTimer planningFailedTimer = new SimpleTimer();

   public LookAndStepBodyPathPart(BehaviorHelper helper,
                                  Supplier<Boolean> isBeingReviewed,
                                  Supplier<Boolean> operatorReviewEnabled,
                                  Supplier<HumanoidRobotState> robotStateSupplier,
                                  LookAndStepBehaviorParameters lookAndStepParameters,
                                  VisibilityGraphsParametersReadOnly visibilityGraphParameters,
                                  Consumer<Consumer<PlanarRegionsListMessage>> mapRegionsCallbackRegistry,
                                  Consumer<TopicListener<Pose3D>> goalInputCallbackRegistry,
                                  Consumer<ArrayList<Pose3D>> successfulPlanConsumer,
                                  Consumer<ArrayList<Pose3D>> review,
                                  Supplier<Boolean> needNewPlan)
   {
      this.helper = helper;
      this.isBeingReviewed = isBeingReviewed;
      this.operatorReviewEnabled = operatorReviewEnabled;
      this.robotStateSupplier = robotStateSupplier;
      this.lookAndStepParameters = lookAndStepParameters;
      this.visibilityGraphParameters = visibilityGraphParameters;
      this.successfulPlanConsumer = successfulPlanConsumer;
      this.review = review;
      this.needNewPlan = needNewPlan;

      // hook up inputs and notifications separately.
      // don't run two body path plans at the same time
      // always run again if with latest data

      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      mapRegionsCallbackRegistry.accept(this::acceptMapRegions);
      mapRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));

      goalInputCallbackRegistry.accept(this::acceptGoal);
      goalInput.addCallback(data -> executor.execute(this::evaluateAndRun));

   }

   private void acceptMapRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      mapRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      mapRegionsExpirationTimer.reset();
   }

   private void acceptGoal(Pose3D goal)
   {
      goalInput.set(goal);
      LogTools.debug("Body path accept goal: {}", goal);
   }

   private void evaluateAndRun()
   {
      // get snapshot of data to process
      SimpleTimer.Status mapRegionsExpirationStatus = mapRegionsExpirationTimer.getStatus(lookAndStepParameters.getPlanarRegionsExpiration());
      SimpleTimer.Status planningFailedTimerStatus
            = planningFailedTimer.getStatus(lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed));
      PlanarRegionsList mapRegions = mapRegionsInput.get();
      Pose3D goal = goalInput.get();
      HumanoidRobotState humanoidRobotState = helper.getOrCreateRobotInterface().pollHumanoidRobotState();

      if (evaluate(helper, mapRegions, goal, mapRegionsExpirationStatus, planningFailedTimerStatus, isBeingReviewed, needNewPlan))
      {
         run(helper,
             mapRegions,
             goal,
             humanoidRobotState,
             visibilityGraphParameters,
             operatorReviewEnabled,
             successfulPlanConsumer,
             () -> planningFailedTimer.reset(),
             review);
      }
   }

   // static so it must use data snapshot
   private static boolean evaluate(BehaviorHelper helper,
                                   PlanarRegionsList mapRegions,
                                   Pose3D goal,
                                   SimpleTimer.Status mapRegionsExpirationStatus,
                                   SimpleTimer.Status planningFailedTimerStatus,
                                   Supplier<Boolean> isBeingReviewed,
                                   Supplier<Boolean> needNewPlan)
   {
      boolean hasGoal = goal != null && !goal.containsNaN();
      if (!hasGoal)
      {
         LogTools.warn("Body path: does not have goal");
         LogTools.debug("Sending planar regions to UI: {}: {}", LocalDateTime.now(), mapRegions.hashCode());
         helper.publishToUI(MapRegionsForUI, mapRegions);
         return false;
      }

      boolean regionsOK = mapRegions != null
                          && !mapRegionsExpirationStatus.isPastOrNaN()
                          && !mapRegions.isEmpty();
      if (!regionsOK)
      {
         LogTools.warn("Body path: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                       mapRegions,
                       mapRegionsExpirationStatus.getTimePassedSinceReset(),
                       mapRegions == null ? null : mapRegions.isEmpty());
         return false;
      }

      if (!needNewPlan.get())
      {
         LogTools.warn("Body path: New plan not needed");
         return false;
      }

      // TODO: This could be "run recently" instead of failed recently
      boolean failedRecently = !planningFailedTimerStatus.isPastOrNaN();
      if (failedRecently)
      {
         LogTools.warn("Body path: failedRecently = true");
         return false;
      }

      if (isBeingReviewed.get())
      {
         LogTools.debug("Body path: bodyPathBeingReviewed = true");
         return false;
      }

      return true;
   }

   private static void run(BehaviorHelper helper,
                           PlanarRegionsList mapRegions,
                           Pose3D goal,
                           HumanoidRobotState humanoidRobotState,
                           VisibilityGraphsParametersReadOnly visibilityGraphParameters,
                           Supplier<Boolean> operatorReviewEnabled,
                           Consumer<ArrayList<Pose3D>> successfulPlanConsumer,
                           Runnable resetPlanningFailedTimer,
                           Consumer<ArrayList<Pose3D>> review)
   {
      // TODO: Add robot standing still for 20s for real robot

      helper.publishToUI(MapRegionsForUI, mapRegions);

      LogTools.info("Planning body path...");

      // calculate and send body path plan
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters,
                                                                                  pathPostProcessor,
                                                                                  new YoVariableRegistry(LookAndStepBodyPathPart.class.getSimpleName()));

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
      bodyPathPlanner.planWaypoints();
      LogTools.info("Body path planning took {}", stopwatch.totalElapsed()); // 0.1 s
      //      bodyPathPlan = bodyPathPlanner.getWaypoints();
      if (bodyPathPlanner.getWaypoints() != null)
      {
         for (Pose3DReadOnly poseWaypoint : bodyPathPlanner.getWaypoints())
         {
            bodyPathPlanForReview.add(new Pose3D(poseWaypoint));
         }
         helper.publishToUI(BodyPathPlanForUI, bodyPathPlanForReview);
      }

      if (bodyPathPlanForReview.size() >= 2)
      {
         if (operatorReviewEnabled.get())
         {
             review.accept(bodyPathPlanForReview);
         }
         else
         {
            successfulPlanConsumer.accept(bodyPathPlanForReview);
         }
      }
      else
      {
         resetPlanningFailedTimer.run();
      }
   }
}
