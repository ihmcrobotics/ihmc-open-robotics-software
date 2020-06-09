package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class LiveMapAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("LiveMapRoot");
   private static final CategoryTheme LiveMap = apiFactory.createCategoryTheme("LiveMap");

   public static final Topic<PlanarRegionSLAMParameters> PlanarRegionSLAMParameters = topic("PlanarRegionSLAMParameters");

   public static final Topic<PlanarRegionsList> LocalizedMap = topic("LocalizedMap");
   public static final Topic<PlanarRegionsList> RegionsAtFeet = topic("RegionsAtFeet");

   public static final Topic<PlanarRegionsList> CombinedLiveMap = topic("CombinedLiveMap");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(LiveMap).topic(apiFactory.createTypedTopicTheme(name));
   }
}
