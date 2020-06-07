package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.Timer;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;
import us.ihmc.humanoidBehaviors.lookAndStep.TypedInput;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.function.Supplier;

/**
 * Needs to be set up as a machine that:
 * - Calls task update as a predefined llambda
 */
public class LookAndStepFootstepPlanningModule extends LookAndStepFootstepPlanningTask
{
   private Field<Supplier<HumanoidRobotState>> robotStateSupplier = required();
   private Field<Supplier<RobotSide>> lastStanceSideSupplier = required();
   private Field<Supplier<LookAndStepBehavior.State>> behaviorStateSupplier = required();

   private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
   private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
   private Timer planarRegionsExpirationTimer = new Timer();
   private Timer planningFailedTimer = new Timer();

   public LookAndStepFootstepPlanningModule()
   {
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      planarRegionsInput.addCallback(data -> executor.execute(this::evaluateAndRun));
      bodyPathPlanInput.addCallback(data -> executor.execute(this::evaluateAndRun));

      setPlanningFailedNotifier(planningFailedTimer::reset);
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

      update(planarRegionsInput.get(),
             planarRegionsExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.get().getPlanarRegionsExpiration()),
             planningFailedTimer.createSnapshot(lookAndStepBehaviorParameters.get().getWaitTimeAfterPlanFailed()),
             bodyPathPlanInput.get(),
             robotStateSupplier.get().get(),
             lastStanceSideSupplier.get().get(),
             behaviorStateSupplier.get().get());

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

   public void setBehaviorStateSupplier(Supplier<LookAndStepBehavior.State> behaviorStateSupplier)
   {
      this.behaviorStateSupplier.set(behaviorStateSupplier);
   }
}
