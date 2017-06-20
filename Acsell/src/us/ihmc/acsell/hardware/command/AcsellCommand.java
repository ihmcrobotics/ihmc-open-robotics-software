package us.ihmc.acsell.hardware.command;

import java.nio.ByteBuffer;
import java.util.EnumMap;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.acsell.hardware.AcsellJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class AcsellCommand<ACTUATOR extends Enum<ACTUATOR> & AcsellActuator, JOINT extends Enum<JOINT> & AcsellJoint>
{

   protected final YoVariableRegistry registry;
   protected final EnumMap<JOINT, AcsellJointCommand> jointCommands;
   protected final EnumMap<ACTUATOR, AcsellActuatorCommand> actuatorCommands;

   public AcsellCommand(String name, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      jointCommands = createJointCommands();
      actuatorCommands = createActuatorCommands();

      parentRegistry.addChild(registry);

   }


   protected abstract EnumMap<JOINT, AcsellJointCommand> createJointCommands();
   protected abstract EnumMap<ACTUATOR, AcsellActuatorCommand> createActuatorCommands();
   
   protected abstract ACTUATOR[] getActuators();
   protected abstract JOINT[] getJoints();

   public double getAcutatorTau(ACTUATOR actuator)
   {
      return actuatorCommands.get(actuator).getTauDesired();
   }

   public void updateActuatorCommandsFromJointCommands()
   {
      for (ACTUATOR actuator : getActuators())
      {
         actuatorCommands.get(actuator).update();
      }
   }

   public void write(ByteBuffer target, int controlID)
   {
      updateActuatorCommandsFromJointCommands();
      for (ACTUATOR actuator : getActuators())
      {
         actuatorCommands.get(actuator).write(target, controlID);
      }
   
   }

   public AcsellJointCommand getAcsellJointCommand(JOINT joint)
   {
      return jointCommands.get(joint);
   }

   public void enableActuators()
   {
      for (ACTUATOR actuator : getActuators())
      {
         actuatorCommands.get(actuator).enable();
      }
   }

   public void disableActuators()
   {
      for (ACTUATOR actuator : getActuators())
      {
         actuatorCommands.get(actuator).disable();
      }
   }

}
