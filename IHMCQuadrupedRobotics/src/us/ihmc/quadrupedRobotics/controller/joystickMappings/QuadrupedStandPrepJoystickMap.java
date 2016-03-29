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
import us.ihmc.tools.inputDevices.joystick.mapping.Thrustmaster16000M;

public class QuadrupedStandPrepJoystickMap implements EnumDependentJoystickMapping<QuadrupedControllerState>
{
   private final ArrayList<JoystickEventListener> eventListeners = new ArrayList<>();
   private final QuadrupedControllerState controllerEnum = QuadrupedControllerState.STAND_PREP;
   private final JoystickToYoVariableMapper joystickToYoVariableMapper;

   private final Component button1;
   private final Component button2;
   private final Component button3;
   private final Component button4;
   
   public QuadrupedStandPrepJoystickMap(final YoVariableHolder yoVariableHolder,
         EnumYoVariableDependentInputManager<QuadrupedControllerState> joystickManager)
   {
      joystickToYoVariableMapper = new JoystickToYoVariableMapper(yoVariableHolder, eventListeners);
      
      final Joystick joystick = joystickManager.getFirstJoystick();
      switch (joystick.getModel())
      {

      case LOGITECH_EXTREME_3D:
         button1 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_6.getIdentifier());
         button2 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_7.getIdentifier());
         button3 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_8.getIdentifier());
         button4 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_9.getIdentifier());
         break;
      case MAD_CATZ_FLY5_STICK:
         button1 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_6.getIdentifier());
         button2 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_7.getIdentifier());
         button3 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_8.getIdentifier());
         button4 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_9.getIdentifier());
         break;
      case THRUSTMASTER_16000M:
         button1 = joystick.findComponent(Thrustmaster16000M.BASE_BUTTON_LEFT_SIDE_UPPER_LEFT.getIdentifier());
         button2 = joystick.findComponent(Thrustmaster16000M.BASE_BUTTON_LEFT_SIDE_UPPER_CENTER.getIdentifier());
         button3 = joystick.findComponent(Thrustmaster16000M.BASE_BUTTON_LEFT_SIDE_UPPER_RIGHT.getIdentifier());
         button4 = joystick.findComponent(Thrustmaster16000M.BASE_BUTTON_LEFT_SIDE_LOWER_LEFT.getIdentifier());
         break;
      case MAD_CATZ_V1_STICK:
         button1 = joystick.findComponent(MadCatzV1StickMapping.BUTTON_6.getIdentifier());
         button2 = null;
         button3 = null;
         button4 = null;
         break;
      case SAITEK_X52:
         button1 = null;
         button2 = null;
         button3 = null;
         button4 = null;
         break;
      case UNKNOWN:
         button1 = null;
         button2 = null;
         button3 = null;
         button4 = null;
         break;
      default:
         button1 = null;
         button2 = null;
         button3 = null;
         button4 = null;
         break;
      }
      
      String controllerStateEnumName = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.QuadrupedControllerStateMachineRequestedState";
      joystickToYoVariableMapper.mapEnumYoVariableToComponent(button3, controllerStateEnumName, QuadrupedControllerState.POSITION_CRAWL);
      joystickToYoVariableMapper.mapEnumYoVariableToComponent(button4, controllerStateEnumName, QuadrupedControllerState.TROT_WALK);
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
