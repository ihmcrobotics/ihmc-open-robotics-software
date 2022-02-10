package us.ihmc.behaviors.heightMapNavigation;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class HeightMapNavigationBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("HeightMapNavigationRoot");
   private static final MessagerAPIFactory.CategoryTheme TargetFollower = apiFactory.createCategoryTheme("HeightMapNavigation");

   public static final MessagerAPIFactory.Topic<Boolean> RequestStart = topic("RequestStart");
   public static final MessagerAPIFactory.Topic<Boolean> RequestStop = topic("RequestStop");
   public static final MessagerAPIFactory.Topic<Pose3D> GoalPose = topic("GoalPose");
   public static final MessagerAPIFactory.Topic<List<String>> HeightMapNavigationParameters = topic("HeightMapNavigationParameters");
   public static final MessagerAPIFactory.Topic<PlanarRegionsList> PlanarRegionsForUI = topic("PlanarRegionsForUI");
   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(TargetFollower).topic(apiFactory.createTypedTopicTheme(name));
   }
}
