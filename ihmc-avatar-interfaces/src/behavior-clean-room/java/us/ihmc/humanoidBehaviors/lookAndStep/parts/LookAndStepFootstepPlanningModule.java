package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;
import us.ihmc.humanoidBehaviors.lookAndStep.TypedInput;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.function.Supplier;

public class LookAndStepFootstepPlanningModule extends LookAndStepFootstepPlanningTask
{
   private Field<Supplier<HumanoidRobotState>> robotStateSupplier = required();
   private Field<Supplier<RobotSide>> lastStanceSideSupplier = required();

   private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
   private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
   private SimpleTimer planarRegionsExpirationTimer = new SimpleTimer();
   private SimpleTimer planningFailedTimer = new SimpleTimer();

   public LookAndStepFootstepPlanningModule()
   {
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      planarRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));
      bodyPathPlanInput.addCallback(data -> executor.execute(this::evaluateAndRun));
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
      validateNonChanging();

      setPlanarRegions(planarRegionsInput.get());
      setBodyPathPlan(bodyPathPlanInput.get());
      setRobotState(robotStateSupplier.get().get());
      setLastStanceSide(lastStanceSideSupplier.get().get());
      setPlanarRegionsExpirationStatus(planarRegionsExpirationTimer.getStatus(lookAndStepBehaviorParameters.get().getPlanarRegionsExpiration()));
      setModuleFailedTimerStatus(planningFailedTimer.getStatus(lookAndStepBehaviorParameters.get().getWaitTimeAfterPlanFailed()));

      run();
   }

   public void setLastStanceSideSupplier(Supplier<RobotSide> lastStanceSide)
   {
      this.lastStanceSideSupplier.set(lastStanceSide);
   }

   public void setRobotStateSupplier(Supplier<HumanoidRobotState> robotStateSupplier)
   {
      this.robotStateSupplier.set(robotStateSupplier);
   }
}
