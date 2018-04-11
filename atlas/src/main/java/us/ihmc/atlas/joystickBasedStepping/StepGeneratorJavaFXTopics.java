package us.ihmc.atlas.joystickBasedStepping;

import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Category;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TypedTopicTheme;

public class StepGeneratorJavaFXTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("StepController"));

   private static final CategoryTheme XBoxOneController = apiFactory.createCategoryTheme("XBoxOneController");
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

   public static final Topic<ButtonState> ButtonAState = Root.child(XBoxOneController).child(Button).child(A).topic(State);
   public static final Topic<ButtonState> ButtonBState = Root.child(XBoxOneController).child(Button).child(B).topic(State);
   public static final Topic<ButtonState> ButtonXState = Root.child(XBoxOneController).child(Button).child(X).topic(State);
   public static final Topic<ButtonState> ButtonYState = Root.child(XBoxOneController).child(Button).child(Y).topic(State);
   public static final Topic<ButtonState> ButtonLeftBumperState = Root.child(XBoxOneController).child(Button).child(LeftBumper).topic(State);
   public static final Topic<ButtonState> ButtonRightBumperState = Root.child(XBoxOneController).child(Button).child(RightBumper).topic(State);
   public static final Topic<ButtonState> ButtonSelectState = Root.child(XBoxOneController).child(Button).child(Select).topic(State);
   public static final Topic<ButtonState> ButtonStartState = Root.child(XBoxOneController).child(Button).child(Start).topic(State);
   public static final Topic<ButtonState> ButtonXBoxState = Root.child(XBoxOneController).child(Button).child(XBox).topic(State);
   public static final Topic<ButtonState> ButtonLeftStickState = Root.child(XBoxOneController).child(Button).child(LeftStick).topic(State);
   public static final Topic<ButtonState> ButtonRightStickState = Root.child(XBoxOneController).child(Button).child(RightStick).topic(State);

   public static final Topic<Double> LeftStickXAxis = Root.child(XBoxOneController).child(LeftStick).child(X).topic(Axis);
   public static final Topic<Double> LeftStickYAxis = Root.child(XBoxOneController).child(LeftStick).child(Y).topic(Axis);
   public static final Topic<Double> RightStickXAxis = Root.child(XBoxOneController).child(RightStick).child(X).topic(Axis);
   public static final Topic<Double> RightStickYAxis = Root.child(XBoxOneController).child(RightStick).child(Y).topic(Axis);
   public static final Topic<Double> LeftTriggerAxis = Root.child(XBoxOneController).child(LeftTrigger).topic(Axis);
   public static final Topic<Double> RightTriggerAxis = Root.child(XBoxOneController).child(RightTrigger).topic(Axis);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
