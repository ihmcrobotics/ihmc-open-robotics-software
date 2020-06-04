package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;
import us.ihmc.humanoidBehaviors.lookAndStep.TypedInput;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequest;

import java.util.function.Supplier;

public class LookAndStepRobotMotionModule extends LookAndStepRobotMotionTask
{
   private final Field<Supplier<HumanoidRobotState>> robotStateSupplier = required();

   private final TypedInput<RobotWalkRequest> robotWalkRequestInput = new TypedInput<>();

   public LookAndStepRobotMotionModule()
   {
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      robotWalkRequestInput.addCallback(data -> executor.execute(this::evaluateAndRun));
   }

   public void acceptRobotWalkRequest(RobotWalkRequest robotWalkRequest)
   {
      robotWalkRequestInput.set(robotWalkRequest); // TODO: There could be data threading error here, might need to queue this data for use in the thread
   }

   private void evaluateAndRun()
   {
      validateNonChanging();

      // set changing stuff
      setFootstepPlan(robotWalkRequestInput.get().getFootstepPlan());
      setPlanarRegions(robotWalkRequestInput.get().getPlanarRegions());
      setRobotState(robotStateSupplier.get().get());

      run();
   }

   public void setRobotStateSupplier(Supplier<HumanoidRobotState> robotStateSupplier)
   {
      this.robotStateSupplier.set(robotStateSupplier);
   }
}
