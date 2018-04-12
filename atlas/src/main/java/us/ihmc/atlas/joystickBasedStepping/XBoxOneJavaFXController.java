package us.ihmc.atlas.joystickBasedStepping;

import net.java.games.input.Event;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Category;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class XBoxOneJavaFXController
{
   public static Topic<ButtonState> ButtonAState, ButtonBState, ButtonXState, ButtonYState;
   public static Topic<ButtonState> ButtonLeftBumperState, ButtonRightBumperState;
   public static Topic<ButtonState> ButtonSelectState, ButtonStartState;
   public static Topic<ButtonState> ButtonXBoxState;
   public static Topic<ButtonState> ButtonLeftStickState, ButtonRightStickState;

   public static Topic<Double> LeftStickXAxis, LeftStickYAxis;
   public static Topic<Double> RightStickXAxis, RightStickYAxis;
   public static Topic<Double> LeftTriggerAxis, RightTriggerAxis;

   public static void addTopicsToAPI(MessagerAPIFactory apiFactory, Category rootCategory)
   {
      CategoryTheme XBoxOneController = apiFactory.createCategoryTheme("XBoxOneController");
      CategoryTheme Button = apiFactory.createCategoryTheme("Button");
      CategoryTheme A = apiFactory.createCategoryTheme("A");
      CategoryTheme B = apiFactory.createCategoryTheme("B");
      CategoryTheme X = apiFactory.createCategoryTheme("X");
      CategoryTheme Y = apiFactory.createCategoryTheme("Y");
      CategoryTheme LeftBumper = apiFactory.createCategoryTheme("LeftBumper");
      CategoryTheme RightBumper = apiFactory.createCategoryTheme("RightBumper");
      CategoryTheme Select = apiFactory.createCategoryTheme("Select");
      CategoryTheme Start = apiFactory.createCategoryTheme("Start");
      CategoryTheme XBox = apiFactory.createCategoryTheme("XBox");
      CategoryTheme LeftTrigger = apiFactory.createCategoryTheme("LeftTrigger");
      CategoryTheme RightTrigger = apiFactory.createCategoryTheme("RightTrigger");
      CategoryTheme LeftStick = apiFactory.createCategoryTheme("LeftStick");
      CategoryTheme RightStick = apiFactory.createCategoryTheme("RightStick");

      TypedTopicTheme<ButtonState> State = apiFactory.createTypedTopicTheme("State");
      TypedTopicTheme<Double> Axis = apiFactory.createTypedTopicTheme("Axis");

      ButtonAState = rootCategory.child(XBoxOneController).child(Button).child(A).topic(State);
      ButtonBState = rootCategory.child(XBoxOneController).child(Button).child(B).topic(State);
      ButtonXState = rootCategory.child(XBoxOneController).child(Button).child(X).topic(State);
      ButtonYState = rootCategory.child(XBoxOneController).child(Button).child(Y).topic(State);
      ButtonLeftBumperState = rootCategory.child(XBoxOneController).child(Button).child(LeftBumper).topic(State);
      ButtonRightBumperState = rootCategory.child(XBoxOneController).child(Button).child(RightBumper).topic(State);
      ButtonSelectState = rootCategory.child(XBoxOneController).child(Button).child(Select).topic(State);
      ButtonStartState = rootCategory.child(XBoxOneController).child(Button).child(Start).topic(State);
      ButtonXBoxState = rootCategory.child(XBoxOneController).child(Button).child(XBox).topic(State);
      ButtonLeftStickState = rootCategory.child(XBoxOneController).child(Button).child(LeftStick).topic(State);
      ButtonRightStickState = rootCategory.child(XBoxOneController).child(Button).child(RightStick).topic(State);

      LeftStickXAxis = rootCategory.child(XBoxOneController).child(LeftStick).child(X).topic(Axis);
      LeftStickYAxis = rootCategory.child(XBoxOneController).child(LeftStick).child(Y).topic(Axis);
      RightStickXAxis = rootCategory.child(XBoxOneController).child(RightStick).child(X).topic(Axis);
      RightStickYAxis = rootCategory.child(XBoxOneController).child(RightStick).child(Y).topic(Axis);
      LeftTriggerAxis = rootCategory.child(XBoxOneController).child(LeftTrigger).topic(Axis);
      RightTriggerAxis = rootCategory.child(XBoxOneController).child(RightTrigger).topic(Axis);
   }

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
         break;
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
