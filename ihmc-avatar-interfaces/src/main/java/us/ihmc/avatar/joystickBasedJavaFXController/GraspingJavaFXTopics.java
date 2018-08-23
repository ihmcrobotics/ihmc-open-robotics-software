package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Category;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TypedTopicTheme;

public class GraspingJavaFXTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("Grasping"));

   private static final CategoryTheme FingerControl = apiFactory.createCategoryTheme("FingerControl");

   private static final CategoryTheme Left = apiFactory.createCategoryTheme("Left");
   private static final CategoryTheme Right = apiFactory.createCategoryTheme("Right");
   private static final CategoryTheme Thumb = apiFactory.createCategoryTheme("Thumb");
   private static final CategoryTheme Index = apiFactory.createCategoryTheme("Index");
   private static final CategoryTheme Middle = apiFactory.createCategoryTheme("Middle");
   private static final CategoryTheme Pinky = apiFactory.createCategoryTheme("Pinky");

   private static final TypedTopicTheme<Double> Roll = apiFactory.createTypedTopicTheme("Roll");
   private static final TypedTopicTheme<Double> Pitch = apiFactory.createTypedTopicTheme("Pitch");
   private static final TypedTopicTheme<Double> Pitch2 = apiFactory.createTypedTopicTheme("Pitch2");
   private static final TypedTopicTheme<Boolean> SendMessage = apiFactory.createTypedTopicTheme("SendMessage");

   private static final TypedTopicTheme<Integer> Selected = apiFactory.createTypedTopicTheme("Selected");

   public static final Topic<Double> LeftThumbRoll = Root.child(FingerControl).child(Left).child(Thumb).topic(Roll);
   public static final Topic<Double> LeftThumb = Root.child(FingerControl).child(Left).child(Thumb).topic(Pitch);
   public static final Topic<Double> LeftThumb2 = Root.child(FingerControl).child(Left).child(Thumb).topic(Pitch2);
   public static final Topic<Double> LeftIndex = Root.child(FingerControl).child(Left).child(Index).topic(Pitch);
   public static final Topic<Double> LeftMiddle = Root.child(FingerControl).child(Left).child(Middle).topic(Pitch);
   public static final Topic<Double> LeftPinky = Root.child(FingerControl).child(Left).child(Pinky).topic(Pitch);
   public static final Topic<Double> RightThumbRoll = Root.child(FingerControl).child(Right).child(Thumb).topic(Roll);
   public static final Topic<Double> RightThumb = Root.child(FingerControl).child(Right).child(Thumb).topic(Pitch);
   public static final Topic<Double> RightThumb2 = Root.child(FingerControl).child(Right).child(Thumb).topic(Pitch2);
   public static final Topic<Double> RightIndex = Root.child(FingerControl).child(Right).child(Index).topic(Pitch);
   public static final Topic<Double> RightMiddle = Root.child(FingerControl).child(Right).child(Middle).topic(Pitch);
   public static final Topic<Double> RightPinky = Root.child(FingerControl).child(Right).child(Pinky).topic(Pitch);
   public static final Topic<Boolean> LeftSendMessage = Root.child(FingerControl).child(Left).topic(SendMessage);
   public static final Topic<Boolean> RightSendMessage = Root.child(FingerControl).child(Right).topic(SendMessage);

   static
   {
      apiFactory.includeMessagerAPIs(XBoxOneJavaFXController.XBoxOneControllerAPI);
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}