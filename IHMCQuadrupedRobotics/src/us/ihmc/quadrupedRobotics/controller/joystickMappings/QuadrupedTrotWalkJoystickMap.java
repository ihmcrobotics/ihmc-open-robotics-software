package us.ihmc.quadrupedRobotics.controller.joystickMappings;

import java.util.ArrayList;

import net.java.games.input.Component;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.simulationconstructionset.joystick.EnumDependentJoystickMapping;
import us.ihmc.simulationconstructionset.joystick.EnumYoVariableDependentInputManager;
import us.ihmc.simulationconstructionset.joystick.JoystickToYoVariableMapper;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.LogitechExtreme3DMapping;
import us.ihmc.tools.inputDevices.joystick.mapping.MadCatzFLY5StickMapping;
import us.ihmc.tools.inputDevices.joystick.mapping.MadCatzV1StickMapping;
import us.ihmc.tools.inputDevices.joystick.mapping.SaitekX52Mapping;

public class QuadrupedTrotWalkJoystickMap implements EnumDependentJoystickMapping<QuadrupedControllerState>
{
   private static final double DEFAULT_DEAD_ZONE = 0.05;
   private static final double BODY_MAX_PITCH = 0.1;
   private static final double BODY_MAX_ROLL = 0.2;
   private static final double BODY_MAX_YAW = 0.03;
   private final ArrayList<JoystickEventListener> eventListeners = new ArrayList<>();
   private final QuadrupedControllerState controllerEnum = QuadrupedControllerState.TROT_WALK;
   private final JoystickToYoVariableMapper joystickToYoVariableMapper;
   private final Joystick joystick;
   
   private final Component rightStickYawComponent;
   private final Component rightStickPitchComponent;
   private final Component rightStickRollComponent;

   public QuadrupedTrotWalkJoystickMap(final YoVariableHolder yoVariableHolder,
         EnumYoVariableDependentInputManager<QuadrupedControllerState> joystickManager)
   {
      joystickToYoVariableMapper = new JoystickToYoVariableMapper(yoVariableHolder, eventListeners);
      
      joystick = joystickManager.getFirstJoystick();
      switch (joystick.getModel())
      {
      case LOGITECH_EXTREME_3D:
         rightStickYawComponent = joystick.findComponent(LogitechExtreme3DMapping.STICK_YAW.getIdentifier());
         rightStickPitchComponent = joystick.findComponent(LogitechExtreme3DMapping.STICK_PITCH.getIdentifier());
         rightStickRollComponent = joystick.findComponent(LogitechExtreme3DMapping.STICK_ROLL.getIdentifier());
         break;
      case MAD_CATZ_FLY5_STICK:
         rightStickYawComponent = joystick.findComponent(MadCatzFLY5StickMapping.STICK_YAW.getIdentifier());
         rightStickPitchComponent = joystick.findComponent(MadCatzFLY5StickMapping.STICK_PITCH.getIdentifier());
         rightStickRollComponent = joystick.findComponent(MadCatzFLY5StickMapping.STICK_ROLL.getIdentifier());
         break;
      case MAD_CATZ_V1_STICK:
         rightStickYawComponent = joystick.findComponent(MadCatzV1StickMapping.STICK_YAW.getIdentifier());
         rightStickPitchComponent = joystick.findComponent(MadCatzV1StickMapping.STICK_PITCH.getIdentifier());
         rightStickRollComponent = joystick.findComponent(MadCatzV1StickMapping.STICK_ROLL.getIdentifier());
         break;
      case SAITEK_X52:
         rightStickYawComponent = joystick.findComponent(SaitekX52Mapping.STICK_YAW.getIdentifier());
         rightStickPitchComponent = joystick.findComponent(SaitekX52Mapping.STICK_PITCH.getIdentifier());
         rightStickRollComponent = joystick.findComponent(SaitekX52Mapping.STICK_ROLL.getIdentifier());
         break;
      case UNKNOWN:
         rightStickYawComponent = joystick.findComponent(Component.Identifier.Axis.RZ);
         rightStickPitchComponent = joystick.findComponent(Component.Identifier.Axis.Y);
         rightStickRollComponent = joystick.findComponent(Component.Identifier.Axis.X);
         break;
      default:
         rightStickYawComponent = null;
         rightStickPitchComponent = null;
         rightStickRollComponent = null;
         break;
      }

      String pitchName = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.TrotWalkController.q_d_pitch";
      joystickToYoVariableMapper.mapDoubleYoVariableToComponent(rightStickPitchComponent, pitchName, -BODY_MAX_PITCH, BODY_MAX_PITCH, DEFAULT_DEAD_ZONE, true);
      String rollName = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.TrotWalkController.q_d_roll";
      joystickToYoVariableMapper.mapDoubleYoVariableToComponent(rightStickRollComponent, rollName, -BODY_MAX_ROLL, BODY_MAX_ROLL, DEFAULT_DEAD_ZONE, false);
      String yawName = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.TrotWalkController.q_d_yaw";
      joystickToYoVariableMapper.mapDoubleYoVariableToComponent(rightStickYawComponent, yawName, -BODY_MAX_YAW, BODY_MAX_YAW, DEFAULT_DEAD_ZONE, false);
   }

   @Override
   public ArrayList<JoystickEventListener> getEventListeners()
   {
      return eventListeners;
   }

   @Override
   public QuadrupedControllerState getEnum()
   {
      return controllerEnum;
   }
}
