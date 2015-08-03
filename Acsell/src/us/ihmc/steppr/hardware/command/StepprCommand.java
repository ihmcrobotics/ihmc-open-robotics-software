package us.ihmc.steppr.hardware.command;

import java.util.EnumMap;

import us.ihmc.acsell.fourbar.FourbarProperties;
import us.ihmc.acsell.hardware.command.AcsellActuatorCommand;
import us.ihmc.acsell.hardware.command.AcsellAnkleActuatorCommand;
import us.ihmc.acsell.hardware.command.AcsellCommand;
import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.command.AcsellKneeActuatorCommand;
import us.ihmc.acsell.hardware.command.AcsellLinearTransmissionActuatorCommand;
import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.configuration.StepprAnkleKinematicParameters;
import us.ihmc.steppr.hardware.configuration.StepprFourbarProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprCommand extends AcsellCommand<StepprActuator, StepprJoint>
{
   public StepprCommand(YoVariableRegistry parentRegistry)
   {
      super("StepprCommand", parentRegistry);
   }

   protected EnumMap<StepprActuator,AcsellActuatorCommand> createActuatorCommands()
   {
      EnumMap<StepprActuator, AcsellActuatorCommand> actuatorCommands = new EnumMap<>(StepprActuator.class);
      
      for (StepprJoint joint : StepprJoint.values)
      {
         if (joint.isLinear())
         {
            StepprActuator actuator = joint.getActuators()[0];
            AcsellLinearTransmissionActuatorCommand actuatorCommand = new AcsellLinearTransmissionActuatorCommand(actuator.getName(), actuator,
                  joint.getRatio(), jointCommands.get(joint), registry);
            actuatorCommands.put(actuator, actuatorCommand);
         }
      }
      FourbarProperties fourbarProperties = new StepprFourbarProperties();

      AcsellKneeActuatorCommand leftKnee = new AcsellKneeActuatorCommand(fourbarProperties, "leftKneeActuator", RobotSide.LEFT, jointCommands.get(StepprJoint.LEFT_KNEE_Y),
            StepprActuator.LEFT_KNEE, StepprJoint.LEFT_KNEE_Y.getRatio(), registry);
      AcsellKneeActuatorCommand rightKnee = new AcsellKneeActuatorCommand(fourbarProperties, "rightKneeActuator", RobotSide.RIGHT, jointCommands.get(StepprJoint.RIGHT_KNEE_Y),
            StepprActuator.RIGHT_KNEE, StepprJoint.RIGHT_KNEE_Y.getRatio(), registry);
      actuatorCommands.put(StepprActuator.LEFT_KNEE, leftKnee);
      actuatorCommands.put(StepprActuator.RIGHT_KNEE, rightKnee);

      AcsellAnkleKinematicParameters parameters = new StepprAnkleKinematicParameters();

      AcsellAnkleActuatorCommand leftAnkle = new AcsellAnkleActuatorCommand(parameters, "leftAnkleCommand", RobotSide.LEFT, jointCommands.get(StepprJoint.LEFT_ANKLE_Y),
            jointCommands.get(StepprJoint.LEFT_ANKLE_X), StepprActuator.LEFT_ANKLE_RIGHT, StepprActuator.LEFT_ANKLE_LEFT, registry);
      actuatorCommands.put(StepprActuator.LEFT_ANKLE_LEFT, leftAnkle.leftActuatorCommand());
      actuatorCommands.put(StepprActuator.LEFT_ANKLE_RIGHT, leftAnkle.rightActuatorCommand());

      AcsellAnkleActuatorCommand rightAnkle = new AcsellAnkleActuatorCommand(parameters, "rightAnkleCommand", RobotSide.RIGHT, jointCommands.get(StepprJoint.RIGHT_ANKLE_Y),
            jointCommands.get(StepprJoint.RIGHT_ANKLE_X), StepprActuator.RIGHT_ANKLE_RIGHT, StepprActuator.RIGHT_ANKLE_LEFT, registry);
      actuatorCommands.put(StepprActuator.RIGHT_ANKLE_LEFT, rightAnkle.leftActuatorCommand());
      actuatorCommands.put(StepprActuator.RIGHT_ANKLE_RIGHT, rightAnkle.rightActuatorCommand());
      
      return actuatorCommands;
   }

   protected EnumMap<StepprJoint, AcsellJointCommand> createJointCommands()
   {
      EnumMap<StepprJoint, AcsellJointCommand> jointCommands = new EnumMap<>(StepprJoint.class);
      for (StepprJoint joint : StepprJoint.values)
      {
         AcsellJointCommand jointCommand = new AcsellJointCommand(joint.getSdfName(), joint.getActuators().length, registry);
         jointCommands.put(joint, jointCommand);

      }
      return jointCommands;
   }

   @Override
   protected StepprActuator[] getActuators()
   {
      return StepprActuator.values;
   }

   @Override
   protected StepprJoint[] getJoints()
   {
      return StepprJoint.values;
   }

}
