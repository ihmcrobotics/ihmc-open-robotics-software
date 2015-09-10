package us.ihmc.acsell.hardware.command;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class AcsellLinearTransmissionActuatorCommand extends AcsellActuatorCommand
{
   private final AcsellJointCommand jointCommand;
   private final double ratio;

   public AcsellLinearTransmissionActuatorCommand(String name, AcsellActuator actuator, double ratio, AcsellJointCommand jointCommand, YoVariableRegistry parentRegistry)
   {
      super(name, actuator, parentRegistry);
      this.ratio = ratio;
      this.jointCommand = jointCommand;
   }

   @Override
   public void update()
   {
      setTauDesired(jointCommand.getTauDesired() / ratio);
      setQdd_d(jointCommand.getQdd_d() * ratio);
      setDamping(jointCommand.getDamping() / (ratio * ratio));
   }

}
