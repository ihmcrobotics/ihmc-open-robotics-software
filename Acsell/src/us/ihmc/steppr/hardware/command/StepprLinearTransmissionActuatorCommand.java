package us.ihmc.steppr.hardware.command;

import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprLinearTransmissionActuatorCommand extends StepprActuatorCommand
{
   private final StepprJointCommand jointCommand;
   private final double ratio;

   public StepprLinearTransmissionActuatorCommand(String name, StepprActuator actuator, double ratio, StepprJointCommand jointCommand, YoVariableRegistry parentRegistry)
   {
      super(name, actuator, parentRegistry);
      this.ratio = ratio;
      this.jointCommand = jointCommand;
   }

   @Override
   public void update()
   {
      setTauDesired(jointCommand.getTauDesired() / ratio);
      setDamping(jointCommand.getDamping() / (ratio * ratio));
   }

}
