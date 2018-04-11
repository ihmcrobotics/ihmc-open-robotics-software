package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonAState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonBState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonLeftBumperState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonLeftStickState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonRightBumperState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonRightStickState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonSelectState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonStartState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonXBoxState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonXState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.ButtonYState;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftStickXAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftStickYAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.LeftTriggerAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.RightStickXAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.RightStickYAxis;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.RightTriggerAxis;

import net.java.games.input.Event;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class XBoxOneJavaFXController
{
   private final Messager messager;
   private final Joystick joystick;

   public XBoxOneJavaFXController(Messager messager) throws JoystickNotFoundException
   {
      this.messager = messager;

      joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, true, 0.1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, true, 0.1, 3));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, true, 0.1, 3));

      joystick.addJoystickEventListener(this::consumeEvent);
   }

   private void consumeEvent(Event event)
   {
      switch (XBoxOneMapping.getMapping(event))
      {
      case A:
         messager.submitMessage(ButtonAState, toState(event));
         break;
      case B:
         messager.submitMessage(ButtonBState, toState(event));
         break;
      case X:
         messager.submitMessage(ButtonXState, toState(event));
      case Y:
         messager.submitMessage(ButtonYState, toState(event));
         break;
      case START:
         messager.submitMessage(ButtonStartState, toState(event));
         break;
      case SELECT:
         messager.submitMessage(ButtonSelectState, toState(event));
         break;
      case XBOX_BUTTON:
         messager.submitMessage(ButtonXBoxState, toState(event));
         break;
      case LEFT_STICK_BUTTON:
         messager.submitMessage(ButtonLeftStickState, toState(event));
         break;
      case RIGHT_STICK_BUTTON:
         messager.submitMessage(ButtonRightStickState, toState(event));
         break;
      case LEFT_BUMPER:
         messager.submitMessage(ButtonLeftBumperState, toState(event));
         break;
      case RIGHT_BUMPER:
         messager.submitMessage(ButtonRightBumperState, toState(event));
         break;
      case LEFT_STICK_X:
         messager.submitMessage(LeftStickXAxis, (double) event.getValue());
         break;
      case LEFT_STICK_Y:
         messager.submitMessage(LeftStickYAxis, (double) event.getValue());
         break;
      case RIGHT_STICK_X:
         messager.submitMessage(RightStickXAxis, (double) event.getValue());
         break;
      case RIGHT_STICK_Y:
         messager.submitMessage(RightStickYAxis, (double) event.getValue());
         break;
      case LEFT_TRIGGER:
         messager.submitMessage(LeftTriggerAxis, (double) event.getValue());
         break;
      case RIGHT_TRIGGER:
         messager.submitMessage(RightTriggerAxis, (double) event.getValue());
         break;

      default:
         break;
      }
   }

   private ButtonState toState(Event event)
   {
      if (event.getComponent().getPollData() > 0.5f)
         return ButtonState.PRESSED;
      else
         return ButtonState.RELEASED;
   }

   public void stop()
   {
      joystick.shutdown();
   }
}
