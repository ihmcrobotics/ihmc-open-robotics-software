package us.ihmc.steppr.hardware.command;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprLinearTransmissionActuatorCommand extends StepprActuatorCommand
{
   private final StepprJointCommand jointCommand;
   private final double ratio;

   public StepprLinearTransmissionActuatorCommand(String name, double kt, double ratio, StepprJointCommand jointCommand, YoVariableRegistry parentRegistry)
   {
      super(name, kt, parentRegistry);
      this.ratio = ratio;
      this.jointCommand = jointCommand;
   }

   @Override
   public void update()
   {
      setTauDesired(jointCommand.getTauDesired() / ratio);
      setDamping(jointCommand.getDamping() / ratio);
   }

}
