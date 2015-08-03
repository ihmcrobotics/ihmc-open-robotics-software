package us.ihmc.wanderer.hardware.command;

import java.util.EnumMap;

import us.ihmc.acsell.fourbar.FourbarProperties;
import us.ihmc.acsell.hardware.command.AcsellActuatorCommand;
import us.ihmc.acsell.hardware.command.AcsellAnkleActuatorCommand;
import us.ihmc.acsell.hardware.command.AcsellCommand;
import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.command.AcsellKneeActuatorCommand;
import us.ihmc.acsell.hardware.command.AcsellLinearTransmissionActuatorCommand;
import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.hardware.configuration.WandererAnkleKinematicParameters;
import us.ihmc.wanderer.hardware.configuration.WandererFourbarProperties;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class WandererCommand extends AcsellCommand<WandererActuator, WandererJoint>
{
   public WandererCommand(YoVariableRegistry parentRegistry)
   {
      super("WandererCommand", parentRegistry);
   }

   protected EnumMap<WandererActuator,AcsellActuatorCommand> createActuatorCommands()
   {
      EnumMap<WandererActuator, AcsellActuatorCommand> actuatorCommands = new EnumMap<>(WandererActuator.class);
      
      for (WandererJoint joint : WandererJoint.values)
      {
         if (joint.isLinear() && !joint.hasNonlinearTransmission())
         {
            WandererActuator actuator = joint.getActuators()[0];
            AcsellLinearTransmissionActuatorCommand actuatorCommand = new AcsellLinearTransmissionActuatorCommand(actuator.getName(), actuator,
                  joint.getRatio(), jointCommands.get(joint), registry);
            actuatorCommands.put(actuator, actuatorCommand);
         }
      }
      FourbarProperties fourbarProperties = new WandererFourbarProperties();

      AcsellKneeActuatorCommand leftKnee = new AcsellKneeActuatorCommand(fourbarProperties, "leftKneeActuator", RobotSide.LEFT, jointCommands.get(WandererJoint.LEFT_KNEE_Y),
            WandererActuator.LEFT_KNEE, WandererJoint.LEFT_KNEE_Y.getRatio(), registry);
      AcsellKneeActuatorCommand rightKnee = new AcsellKneeActuatorCommand(fourbarProperties, "rightKneeActuator", RobotSide.RIGHT, jointCommands.get(WandererJoint.RIGHT_KNEE_Y),
            WandererActuator.RIGHT_KNEE, WandererJoint.RIGHT_KNEE_Y.getRatio(), registry);
      actuatorCommands.put(WandererActuator.LEFT_KNEE, leftKnee);
      actuatorCommands.put(WandererActuator.RIGHT_KNEE, rightKnee);

      AcsellAnkleKinematicParameters parameters = new WandererAnkleKinematicParameters();

      AcsellAnkleActuatorCommand leftAnkle = new AcsellAnkleActuatorCommand(parameters, "leftAnkleCommand", RobotSide.LEFT, jointCommands.get(WandererJoint.LEFT_ANKLE_Y),
            jointCommands.get(WandererJoint.LEFT_ANKLE_X), WandererActuator.LEFT_ANKLE_RIGHT, WandererActuator.LEFT_ANKLE_LEFT, registry);
      actuatorCommands.put(WandererActuator.LEFT_ANKLE_LEFT, leftAnkle.leftActuatorCommand());
      actuatorCommands.put(WandererActuator.LEFT_ANKLE_RIGHT, leftAnkle.rightActuatorCommand());

      AcsellAnkleActuatorCommand rightAnkle = new AcsellAnkleActuatorCommand(parameters, "rightAnkleCommand", RobotSide.RIGHT, jointCommands.get(WandererJoint.RIGHT_ANKLE_Y),
            jointCommands.get(WandererJoint.RIGHT_ANKLE_X), WandererActuator.RIGHT_ANKLE_RIGHT, WandererActuator.RIGHT_ANKLE_LEFT, registry);
      actuatorCommands.put(WandererActuator.RIGHT_ANKLE_LEFT, rightAnkle.leftActuatorCommand());
      actuatorCommands.put(WandererActuator.RIGHT_ANKLE_RIGHT, rightAnkle.rightActuatorCommand());
      
      return actuatorCommands;
   }

   protected EnumMap<WandererJoint, AcsellJointCommand> createJointCommands()
   {
      EnumMap<WandererJoint, AcsellJointCommand> jointCommands = new EnumMap<>(WandererJoint.class);
      for (WandererJoint joint : WandererJoint.values)
      {
         AcsellJointCommand jointCommand = new AcsellJointCommand(joint.getSdfName(), joint.getActuators().length, registry);
         jointCommands.put(joint, jointCommand);

      }
      return jointCommands;
   }

   @Override
   protected WandererActuator[] getActuators()
   {
      return WandererActuator.values;
   }

   @Override
   protected WandererJoint[] getJoints()
   {
      return WandererJoint.values;
   }

}
