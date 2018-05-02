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

   private static final CategoryTheme WalkingController = apiFactory.createCategoryTheme("WalkingController");
   private static final CategoryTheme Swing = apiFactory.createCategoryTheme("Swing");
   private static final CategoryTheme Transfer = apiFactory.createCategoryTheme("Transfer");
   private static final CategoryTheme Trajectory = apiFactory.createCategoryTheme("Trajectory");

   private static final TypedTopicTheme<Double> Height = apiFactory.createTypedTopicTheme("Height");
   private static final TypedTopicTheme<Double> Duration = apiFactory.createTypedTopicTheme("Duration");

   public static final Topic<Double> WalkingSwingHeight = Root.child(WalkingController).child(Swing).topic(Height);
   public static final Topic<Double> WalkingSwingDuration = Root.child(WalkingController).child(Swing).topic(Duration);
   public static final Topic<Double> WalkingTransferDuration = Root.child(WalkingController).child(Transfer).topic(Duration);
   public static final Topic<Double> WalkingTrajectoryDuration = Root.child(WalkingController).child(Trajectory).topic(Duration);

   static
   {
      apiFactory.includeMessagerAPIs(XBoxOneJavaFXController.XBoxOneControllerAPI);
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
