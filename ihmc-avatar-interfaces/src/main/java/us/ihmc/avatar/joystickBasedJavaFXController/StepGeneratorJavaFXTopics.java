package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;

public class StepGeneratorJavaFXTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("StepController"));

   private static final CategoryTheme WalkingController = apiFactory.createCategoryTheme("WalkingController");
   private static final CategoryTheme Trajectory = apiFactory.createCategoryTheme("Trajectory");
   private static final CategoryTheme Stepping = apiFactory.createCategoryTheme("Stepping");

   private static final TypedTopicTheme<Double> Duration = apiFactory.createTypedTopicTheme("Duration");
   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   public static final Topic<Double> WalkingTrajectoryDuration = Root.child(WalkingController).child(Trajectory).topic(Duration);

   public static final Topic<JoystickStepParameters> SteppingParameters = Root.child(Stepping).topic(Parameters);

   static
   {
      apiFactory.includeMessagerAPIs(XBoxOneJavaFXController.XBoxOneControllerAPI);
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
