package us.ihmc.steppr.hardware.command;

import java.nio.ByteBuffer;
import java.util.EnumMap;

import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprCommand
{
   private final YoVariableRegistry registry = new YoVariableRegistry("StepprCommand");

   private final EnumMap<StepprJoint, StepprJointCommand> jointCommands = new EnumMap<>(StepprJoint.class);
   private final EnumMap<StepprActuator, StepprActuatorCommand> actuatorCommands = new EnumMap<>(StepprActuator.class);

   public StepprCommand(YoVariableRegistry parentRegistry)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         StepprJointCommand jointCommand = new StepprJointCommand(joint.getSdfName(), joint.getActuators().length, registry);
         jointCommands.put(joint, jointCommand);

         if (joint.isLinear() )
         {
            StepprActuator actuator = joint.getActuators()[0];
            StepprLinearTransmissionActuatorCommand actuatorCommand = new StepprLinearTransmissionActuatorCommand(actuator.getName(),
                  actuator, joint.getRatio(), jointCommand, registry);
            actuatorCommands.put(actuator, actuatorCommand);
         }
      }

      StepprKneeActuatorCommand leftKnee = new StepprKneeActuatorCommand("leftKneeActuator", jointCommands.get(StepprJoint.LEFT_KNEE_Y), StepprActuator.LEFT_KNEE, StepprJoint.LEFT_KNEE_Y.getRatio(), registry);
      StepprKneeActuatorCommand rightKnee = new StepprKneeActuatorCommand("rightKneeActuator", jointCommands.get(StepprJoint.RIGHT_KNEE_Y), StepprActuator.RIGHT_KNEE, StepprJoint.RIGHT_KNEE_Y.getRatio(), registry);
      actuatorCommands.put(StepprActuator.LEFT_KNEE, leftKnee);
      actuatorCommands.put(StepprActuator.RIGHT_KNEE, rightKnee);
      
      StepprAnkleActuatorCommand leftAnkle = new StepprAnkleActuatorCommand("leftAnkleCommand", jointCommands.get(StepprJoint.LEFT_ANKLE_Y),
            jointCommands.get(StepprJoint.LEFT_ANKLE_X), StepprActuator.LEFT_ANKLE_RIGHT, StepprActuator.LEFT_ANKLE_LEFT, registry);
      actuatorCommands.put(StepprActuator.LEFT_ANKLE_LEFT, leftAnkle.leftActuatorCommand());
      actuatorCommands.put(StepprActuator.LEFT_ANKLE_RIGHT, leftAnkle.rightActuatorCommand());

      StepprAnkleActuatorCommand rightAnkle = new StepprAnkleActuatorCommand("rightAnkleCommand", jointCommands.get(StepprJoint.RIGHT_ANKLE_Y),
            jointCommands.get(StepprJoint.RIGHT_ANKLE_X), StepprActuator.RIGHT_ANKLE_RIGHT, StepprActuator.RIGHT_ANKLE_LEFT, registry);
      actuatorCommands.put(StepprActuator.RIGHT_ANKLE_LEFT, rightAnkle.leftActuatorCommand());
      actuatorCommands.put(StepprActuator.RIGHT_ANKLE_RIGHT, rightAnkle.rightActuatorCommand());

      parentRegistry.addChild(registry);
   }

   public double getAcutatorTau(StepprActuator actuator)
   {
      return actuatorCommands.get(actuator).getTauDesired();
   }

   public void updateActuatorCommandsFromJointCommands()
   {
      for (StepprActuator actuator : StepprActuator.values)
      {
         actuatorCommands.get(actuator).update();
      }
   }

   public void write(ByteBuffer target, int controlID)
   {
      updateActuatorCommandsFromJointCommands();
      for (StepprActuator actuator : StepprActuator.values)
      {
         actuatorCommands.get(actuator).write(target, controlID);
      }

   }

   public StepprJointCommand getStepprJointCommand(StepprJoint joint)
   {
      return jointCommands.get(joint);
   }

   public void enableActuators()
   {
      for (StepprActuator actuator : StepprActuator.values)
      {
         actuatorCommands.get(actuator).enable();
      }
   }

   public void disableActuators()
   {
      for (StepprActuator actuator : StepprActuator.values)
      {
         actuatorCommands.get(actuator).disable();
      }
   }

}
