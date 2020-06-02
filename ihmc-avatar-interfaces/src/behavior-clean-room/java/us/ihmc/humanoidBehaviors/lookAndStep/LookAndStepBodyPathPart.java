package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.log.LogTools;
import us.ihmc.messager.TopicListener;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class LookAndStepBodyPathPart
{
   private final BehaviorHelper helper;
   private final Supplier<Boolean> isBeingReviewed;
   private final Supplier<Boolean> operatorReviewEnabled;
   private final LookAndStepBehaviorParameters lookAndStepParameters;
   private final VisibilityGraphsParametersReadOnly visibilityGraphParameters;
   private final Consumer<ArrayList<Pose3D>> successfulPlanConsumer;
   private final Consumer<ArrayList<Pose3D>> review;
   private final Supplier<Boolean> needNewPlan;

   private final TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
   private SimpleTimer mapRegionsExpirationTimer = new SimpleTimer();

   private final TypedInput<Pose3D> goalInput = new TypedInput<>();

   private SimpleTimer planningFailedTimer = new SimpleTimer();
   private final LookAndStepBodyPathTask task;

   public LookAndStepBodyPathPart(BehaviorHelper helper,
                                  Supplier<Boolean> isBeingReviewed,
                                  Supplier<Boolean> operatorReviewEnabled,
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

      task = new LookAndStepBodyPathTask(helper);
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
      task.setGoal(goalInput.get());
      task.setMapRegions(mapRegionsInput.get());
      task.setHumanoidRobotState(helper.getOrCreateRobotInterface().pollHumanoidRobotState());
      task.setIsBeingReviewed(isBeingReviewed);
      task.setMapRegionsExpirationStatus(mapRegionsExpirationTimer.getStatus(lookAndStepParameters.getPlanarRegionsExpiration()));
      task.setPlanningFailedTimerStatus(planningFailedTimer.getStatus(lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed)));
      task.setResetPlanningFailedTimer(() -> planningFailedTimer.reset());
      task.setReviewInitiator(review);
      task.setSuccessfulPlanConsumer(successfulPlanConsumer);
      task.setVisibilityGraphParameters(visibilityGraphParameters);
      task.setNeedNewPlan(needNewPlan);
      task.setOperatorReviewEnabled(operatorReviewEnabled);

      task.run();
   }
}
