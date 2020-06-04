package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidBehaviors.tools.Builder;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class LookAndStepBodyPathModule implements Builder
{
   // TODO: Could optional be used here for some things to make things more flexible?
   private Field<Supplier<HumanoidRobotState>> robotStateSupplier = required();
   private Field<Supplier<Boolean>> isBeingReviewedSupplier = required();
   private Field<Supplier<Boolean>> reviewEnabledSupplier = required();
   private Field<LookAndStepBehaviorParametersReadOnly> lookAndStepBehaviorParameters = required();
   private Field<VisibilityGraphsParametersReadOnly> visibilityGraphParameters = required();
   private Field<Consumer<List<? extends Pose3DReadOnly>>> initiateReviewOutput = required();
   private Field<Consumer<List<? extends Pose3DReadOnly>>> autonomousOutput = required();
   private Field<Supplier<Boolean>> needNewPlanSupplier = required();
   private Field<UIPublisher> uiPublisher = required();

   private final TypedInput<PlanarRegionsList> mapRegionsInput = new TypedInput<>();
   private final TypedInput<Pose3D> goalInput = new TypedInput<>();
   private SimpleTimer mapRegionsExpirationTimer = new SimpleTimer();
   private SimpleTimer planningFailedTimer = new SimpleTimer();

   private LookAndStepBodyPathTask task;

   // hook up inputs and notifications separately.
   // always run again if with latest data
   public LookAndStepBodyPathModule()
   {
      // don't run two body path plans at the same time
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      mapRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));
      goalInput.addCallback(data -> executor.execute(this::evaluateAndRun));

      task = new LookAndStepBodyPathTask();
   }

   public void acceptMapRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      mapRegionsInput.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      mapRegionsExpirationTimer.reset();
   }

   public void acceptGoal(Pose3D goal)
   {
      goalInput.set(goal);
      LogTools.debug("Body path accept goal: {}", goal);
   }

   private void evaluateAndRun()
   {
      validate();

      task.setGoal(goalInput.get());
      task.setMapRegions(mapRegionsInput.get());
      task.setHumanoidRobotState(robotStateSupplier.get().get());
      task.setVisibilityGraphParameters(visibilityGraphParameters.get());
      task.setMapRegionsExpirationStatus(mapRegionsExpirationTimer.getStatus(lookAndStepBehaviorParameters.get().getPlanarRegionsExpiration()));
      task.setPlanningFailedTimerStatus(planningFailedTimer.getStatus(lookAndStepBehaviorParameters.get().getWaitTimeAfterPlanFailed()));
      task.setResetPlanningFailedTimer(planningFailedTimer::reset);
      task.setReviewInitiator(initiateReviewOutput.get()::accept);
      task.setSuccessfulPlanConsumer(autonomousOutput.get()::accept);
      task.setIsBeingReviewed(isBeingReviewedSupplier.get()); // reactive
      task.setNeedNewPlan(needNewPlanSupplier.get()); // reactive
      task.setOperatorReviewEnabled(reviewEnabledSupplier.get()); //reactive
      task.setUiPublisher(uiPublisher.get());

      task.run();
   }

   public void setRobotStateSupplier(Supplier<HumanoidRobotState> robotStateSupplier)
   {
      this.robotStateSupplier.set(robotStateSupplier);
   }

   public void setIsBeingReviewedSupplier(Supplier<Boolean> isBeingReviewedSupplier)
   {
      this.isBeingReviewedSupplier.set(isBeingReviewedSupplier);
   }

   public void setReviewEnabledSupplier(Supplier<Boolean> reviewEnabledSupplier)
   {
      this.reviewEnabledSupplier.set(reviewEnabledSupplier);
   }

   public void setLookAndStepBehaviorParameters(LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters)
   {
      this.lookAndStepBehaviorParameters.set(lookAndStepBehaviorParameters);
   }

   public void setVisibilityGraphParameters(VisibilityGraphsParametersReadOnly visibilityGraphParameters)
   {
      this.visibilityGraphParameters.set(visibilityGraphParameters);
   }

   public void setInitiateReviewOutput(Consumer<List<? extends Pose3DReadOnly>> initiateReviewOutput)
   {
      this.initiateReviewOutput.set(initiateReviewOutput);
   }

   public void setAutonomousOutput(Consumer<List<? extends Pose3DReadOnly>> autonomousOutput)
   {
      this.autonomousOutput.set(autonomousOutput);
   }

   public void setNeedNewPlanSupplier(Supplier<Boolean> needNewPlanSupplier)
   {
      this.needNewPlanSupplier.set(needNewPlanSupplier);
   }

   public void setUIPublisher(UIPublisher uiPublisher)
   {
      this.uiPublisher.set(uiPublisher);
   }
}
