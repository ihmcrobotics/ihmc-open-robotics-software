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

public class QuadrupedStandPrepJoystickMap implements EnumDependentJoystickMapping<QuadrupedControllerState>
{
   private final ArrayList<JoystickEventListener> eventListeners = new ArrayList<>();
   private final QuadrupedControllerState controllerEnum = QuadrupedControllerState.STAND_PREP;
   private final JoystickToYoVariableMapper joystickToYoVariableMapper;

   private final Component rightButton6;
   private final Component rightButton7;
   private final Component rightButton8;
   private final Component rightButton9;
   
   public QuadrupedStandPrepJoystickMap(final YoVariableHolder yoVariableHolder,
         EnumYoVariableDependentInputManager<QuadrupedControllerState> joystickManager)
   {
      joystickToYoVariableMapper = new JoystickToYoVariableMapper(yoVariableHolder, eventListeners);
      
      final Joystick joystick = joystickManager.getFirstJoystick();
      switch (joystick.getModel())
      {

      case LOGITECH_EXTREME_3D:
         rightButton6 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_6.getIdentifier());
         rightButton7 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_7.getIdentifier());
         rightButton8 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_8.getIdentifier());
         rightButton9 = joystick.findComponent(LogitechExtreme3DMapping.BUTTON_9.getIdentifier());
         break;
      case MAD_CATZ_FLY5_STICK:
         rightButton6 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_6.getIdentifier());
         rightButton7 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_7.getIdentifier());
         rightButton8 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_8.getIdentifier());
         rightButton9 = joystick.findComponent(MadCatzFLY5StickMapping.BUTTON_9.getIdentifier());
         break;
      case MAD_CATZ_V1_STICK:
         rightButton6 = joystick.findComponent(MadCatzV1StickMapping.BUTTON_6.getIdentifier());
         rightButton7 = null;
         rightButton8 = null;
         rightButton9 = null;
         break;
      case SAITEK_X52:
         rightButton6 = null;
         rightButton7 = null;
         rightButton8 = null;
         rightButton9 = null;
         break;
      case UNKNOWN:
         rightButton6 = null;
         rightButton7 = null;
         rightButton8 = null;
         rightButton9 = null;
         break;
      default:
         rightButton6 = null;
         rightButton7 = null;
         rightButton8 = null;
         rightButton9 = null;
         break;
      }
      
      String controllerStateEnumName = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.QuadrupedControllerStateMachineRequestedState";
      joystickToYoVariableMapper.mapEnumYoVariableToComponent(rightButton8, controllerStateEnumName, QuadrupedControllerState.POSITION_CRAWL);
      joystickToYoVariableMapper.mapEnumYoVariableToComponent(rightButton9, controllerStateEnumName, QuadrupedControllerState.TROT_WALK);
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
