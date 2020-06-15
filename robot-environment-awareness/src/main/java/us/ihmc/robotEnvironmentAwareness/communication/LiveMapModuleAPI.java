package us.ihmc.robotEnvironmentAwareness.communication;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class LiveMapModuleAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("LiveMapRoot");
   private static final CategoryTheme LiveMap = apiFactory.createCategoryTheme("LiveMap");

   public static final Topic<Boolean> ViewingEnable = topic("ViewingEnable");
   public static final Topic<Boolean> Clear = topic("Clear");

   public static final Topic<PlanarRegionsListMessage> LocalizedMap = topic("LocalizedMap");
   public static final Topic<PlanarRegionsListMessage> RegionsAtFeet = topic("RegionsAtFeet");
   public static final Topic<PlanarRegionsListMessage> CombinedLiveMap = topic("PlanarRegionsState");

   public static final Topic<PlanarRegionSLAMParameters> PlanarRegionsSLAMParameters = topic("PlanarRegionsSLAMParameters");

   public static final Topic<Boolean> RequestEntireModuleState = topic("RequestEntireModuleState");

   public static final Topic<Boolean> SaveUpdaterConfiguration = topic("SaveUpdaterConfiguration");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(LiveMap).topic(apiFactory.createTypedTopicTheme(name));
   }
}
