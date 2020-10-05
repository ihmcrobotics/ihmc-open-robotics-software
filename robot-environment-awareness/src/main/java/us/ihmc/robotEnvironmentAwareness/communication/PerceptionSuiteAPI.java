package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class PerceptionSuiteAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("PerceptionSuiteRoot"));
   private static final CategoryTheme PerceptionSuite = apiFactory.createCategoryTheme("PerceptionSuite");

   public static final Topic<Boolean> RunRealSenseSLAM = topic("RunRealSenseSLAM");
   public static final Topic<Boolean> RunRealSenseSLAMUI = topic("RunRealSenseSLAMUI");
   public static final Topic<Boolean> GUIRunRealSenseSLAM = topic("GUIRunRealSenseSLAM");
   public static final Topic<Boolean> GUIRunRealSenseSLAMUI = topic("GUIRunRealSenseSLAMUI");

   public static final Topic<Boolean> RunMapSegmentation = topic("RunMapSegmentation");
   public static final Topic<Boolean> RunMapSegmentationUI = topic("RunMapSegmentationUI");
   public static final Topic<Boolean> GUIRunMapSegmentation = topic("GUIRunMapSegmentation");
   public static final Topic<Boolean> GUIRunMapSegmentationUI = topic("GUIRunMapSegmentationUI");

   public static final Topic<Boolean> RunLidarREA = topic("RunLidarREA");
   public static final Topic<Boolean> RunLidarREAUI = topic("RunLidarREAUI");
   public static final Topic<Boolean> GUIRunLidarREA = topic("GUIRunLidarREA");
   public static final Topic<Boolean> GUIRunLidarREAUI = topic("GUIRunLidarREAUI");

   public static final Topic<Boolean> RunRealSenseREA = topic("RunDepthREA");
   public static final Topic<Boolean> RunRealSenseREAUI = topic("RunDepthREAUI");
   public static final Topic<Boolean> GUIRunRealSenseREA = topic("GUIRunDepthREA");
   public static final Topic<Boolean> GUIRunRealSenseREAUI = topic("GUIRunDepthREAUI");

   public static final Topic<Boolean> RunLiveMap = topic("RunLiveMap");
   public static final Topic<Boolean> RunLiveMapUI = topic("RunLiveMapUI");
   public static final Topic<Boolean> GUIRunLiveMap = topic("GUIRunLiveMap");
   public static final Topic<Boolean> GUIRunLiveMapUI = topic("GUIRunLiveMapUI");

   public static final Topic<String> ErrorMessage = topic("ErrorMessage");
   public static final Topic<Boolean> ClearErrorMessage = topic("ClearErrorMessage");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(PerceptionSuite).topic(apiFactory.createTypedTopicTheme(name));
   }
}
