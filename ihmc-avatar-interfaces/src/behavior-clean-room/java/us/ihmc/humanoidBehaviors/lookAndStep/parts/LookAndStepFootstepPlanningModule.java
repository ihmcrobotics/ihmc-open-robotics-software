package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParametersReadOnly;
import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;
import us.ihmc.humanoidBehaviors.lookAndStep.TypedInput;
import us.ihmc.humanoidBehaviors.tools.BehaviorBuilderPattern;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequest;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class LookAndStepFootstepPlanningModule implements BehaviorBuilderPattern
{
   private Field<Supplier<Boolean>> isBeingReviewedSupplier = required();
   private Field<LookAndStepBehaviorParametersReadOnly> lookAndStepBehaviorParameters = required();
   private Field<UIPublisher> uiPublisher = required();
   private Field<Supplier<HumanoidRobotState>> robotStateSupplier = required();
   private Field<FootstepPlannerParametersReadOnly> footstepPlannerParameters = required();
   private Field<Runnable> newBodyPathGoalNeededNotifier = required();
   private Field<Function<RobotSide, FramePose3DReadOnly>> lastSteppedSolePoseSupplier = required();
   private Field<BiConsumer<RobotSide, FramePose3DReadOnly>> lastSteppedSolePoseConsumer = required();
   private Field<Supplier<RobotSide>> lastStanceSide = required();
   private Field<FootstepPlanningModule> footstepPlanningModule = required();
   private Field<Consumer<RobotSide>> lastStanceSideSetter = required();
   private Field<Supplier<Boolean>> operatorReviewEnabledSupplier = required();
   private Field<Consumer<RobotWalkRequest>> reviewPlanOutput = required();
   private Field<Consumer<RobotWalkRequest>> autonomousOutput = required();

   private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
   private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
   private SimpleTimer planarRegionsExpirationTimer = new SimpleTimer();
   private SimpleTimer planningFailedTimer = new SimpleTimer();

   private LookAndStepFootstepPlanningTask task;

   public LookAndStepFootstepPlanningModule()
   {
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      planarRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));
      bodyPathPlanInput.addCallback(data -> executor.execute(this::evaluateAndRun));

      task = new LookAndStepFootstepPlanningTask();
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

   public void evaluateAndRun()
   {
      validate();

      // setters
      task.setPlanarRegions(planarRegionsInput.get());
      task.setPlanarRegionsExpirationStatus(planarRegionsExpirationTimer.getStatus(lookAndStepBehaviorParameters.get().getPlanarRegionsExpiration()));
      task.setModuleFailedTimerStatus(planningFailedTimer.getStatus(lookAndStepBehaviorParameters.get().getWaitTimeAfterPlanFailed()));
      task.setBodyPathPlan(bodyPathPlanInput.get());
      task.setIsBeingReviewedSupplier(isBeingReviewedSupplier.get()); // TODO this pass through, could we set the task directly?
      task.setUiPublisher(uiPublisher.get());
      task.setRobotState(robotStateSupplier.get().get());
      task.setLookAndStepBehaviorParameters(lookAndStepBehaviorParameters.get());
      task.setFootstepPlannerParameters(footstepPlannerParameters.get());
      task.setNewBodyPathGoalNeededNotifier(newBodyPathGoalNeededNotifier.get());
      task.setLastSteppedSolePoseSupplier(lastSteppedSolePoseSupplier.get());
      task.setLastStanceSide(lastStanceSide.get().get());
      task.setLastStanceSideSetter(lastStanceSideSetter.get());
      task.setFootstepPlanningModule(footstepPlanningModule.get());
      task.setOperatorReviewEnabledSupplier(operatorReviewEnabledSupplier.get());
      task.setReviewPlanOutput(reviewPlanOutput.get());
      task.setAutonomousOutput(autonomousOutput.get());

      task.run();
   }

   public void setIsBeingReviewedSupplier(Supplier<Boolean> isBeingReviewedSupplier)
   {
      this.isBeingReviewedSupplier.set(isBeingReviewedSupplier);
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

   public void setLastSteppedSolePoseSupplier(Function<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseSupplier)
   {
      this.lastSteppedSolePoseSupplier.set(lastSteppedSolePoseSupplier);
   }

   public void setLastSteppedSolePoseConsumer(BiConsumer<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseConsumer)
   {
      this.lastSteppedSolePoseConsumer.set(lastSteppedSolePoseConsumer);
   }

   public void setLastStanceSideSupplier(Supplier<RobotSide> lastStanceSide)
   {
      this.lastStanceSide.set(lastStanceSide);
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

   public void setReviewPlanOutput(Consumer<RobotWalkRequest> reviewPlanOutput)
   {
      this.reviewPlanOutput.set(reviewPlanOutput);
   }

   public void setAutonomousOutput(Consumer<RobotWalkRequest> autonomousOutput)
   {
      this.autonomousOutput.set(autonomousOutput);
   }
}
