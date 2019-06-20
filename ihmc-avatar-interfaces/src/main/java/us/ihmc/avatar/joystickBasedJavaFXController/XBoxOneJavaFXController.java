package us.ihmc.avatar.joystickBasedJavaFXController;

import net.java.games.input.Event;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class XBoxOneJavaFXController
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("XBoxOneController"));

   private static final CategoryTheme Button = apiFactory.createCategoryTheme("Button");
   private static final CategoryTheme A = apiFactory.createCategoryTheme("A");
   private static final CategoryTheme B = apiFactory.createCategoryTheme("B");
   private static final CategoryTheme X = apiFactory.createCategoryTheme("X");
   private static final CategoryTheme Y = apiFactory.createCategoryTheme("Y");
   private static final CategoryTheme LeftBumper = apiFactory.createCategoryTheme("LeftBumper");
   private static final CategoryTheme RightBumper = apiFactory.createCategoryTheme("RightBumper");
   private static final CategoryTheme Select = apiFactory.createCategoryTheme("Select");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme XBox = apiFactory.createCategoryTheme("XBox");
   private static final CategoryTheme LeftTrigger = apiFactory.createCategoryTheme("LeftTrigger");
   private static final CategoryTheme RightTrigger = apiFactory.createCategoryTheme("RightTrigger");
   private static final CategoryTheme LeftStick = apiFactory.createCategoryTheme("LeftStick");
   private static final CategoryTheme RightStick = apiFactory.createCategoryTheme("RightStick");

   private static final TypedTopicTheme<ButtonState> State = apiFactory.createTypedTopicTheme("State");
   private static final TypedTopicTheme<Double> Axis = apiFactory.createTypedTopicTheme("Axis");
   private static final TypedTopicTheme<Boolean> Reconnect = apiFactory.createTypedTopicTheme("Reconnect");

   public static final Topic<ButtonState> ButtonAState = Root.child(Button).child(A).topic(State);
   public static final Topic<ButtonState> ButtonBState = Root.child(Button).child(B).topic(State);
   public static final Topic<ButtonState> ButtonXState = Root.child(Button).child(X).topic(State);
   public static final Topic<ButtonState> ButtonYState = Root.child(Button).child(Y).topic(State);
   public static final Topic<ButtonState> ButtonLeftBumperState = Root.child(Button).child(LeftBumper).topic(State);
   public static final Topic<ButtonState> ButtonRightBumperState = Root.child(Button).child(RightBumper).topic(State);
   public static final Topic<ButtonState> ButtonSelectState = Root.child(Button).child(Select).topic(State);
   public static final Topic<ButtonState> ButtonStartState = Root.child(Button).child(Start).topic(State);
   public static final Topic<ButtonState> ButtonXBoxState = Root.child(Button).child(XBox).topic(State);
   public static final Topic<ButtonState> ButtonLeftStickState = Root.child(Button).child(LeftStick).topic(State);
   public static final Topic<ButtonState> ButtonRightStickState = Root.child(Button).child(RightStick).topic(State);

   public static final Topic<Double> LeftStickXAxis = Root.child(LeftStick).child(X).topic(Axis);
   public static final Topic<Double> LeftStickYAxis = Root.child(LeftStick).child(Y).topic(Axis);
   public static final Topic<Double> RightStickXAxis = Root.child(RightStick).child(X).topic(Axis);
   public static final Topic<Double> RightStickYAxis = Root.child(RightStick).child(Y).topic(Axis);
   public static final Topic<Double> LeftTriggerAxis = Root.child(LeftTrigger).topic(Axis);
   public static final Topic<Double> RightTriggerAxis = Root.child(RightTrigger).topic(Axis);

   public static final Topic<Boolean> XBoxControllerReconnect = Root.topic(Reconnect);

   public static final MessagerAPI XBoxOneControllerAPI = apiFactory.getAPIAndCloseFactory();

   private final Messager messager;
   private Joystick joystick;

   public XBoxOneJavaFXController(Messager messager) throws JoystickNotFoundException
   {
      this.messager = messager;

      messager.registerTopicListener(XBoxControllerReconnect, value -> reconnectJoystick());

      connectJoystick();
   }

   public void reconnectJoystick()
   {
      if (joystick != null)
      {
         joystick.clearEventListeners();
         joystick.shutdown();
         joystick = null;
      }

      try
      {
         connectJoystick();
      }
      catch (JoystickNotFoundException e)
      {
         LogTools.warn("Could not reconnect joystick, try again.");
      }
   }

   private void connectJoystick() throws JoystickNotFoundException
   {
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
      if (joystick != null)
         joystick.shutdown();
   }
}
