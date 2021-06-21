package us.ihmc.behaviors.demo;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;

import java.util.List;

public class BuildingExplorationBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("DemoCoordinatorRoot");
   private static final MessagerAPIFactory.CategoryTheme DemoCoordinator = apiFactory.createCategoryTheme("DemoCoordinator");

   public static final MessagerAPIFactory.Topic<BuildingExplorationBehaviorMode> Mode = topic("Mode");

   public static final MessagerAPIFactory.Topic<Pose3D> Goal = topic("Goal");
   public static final MessagerAPIFactory.Topic<Pose3D> GoalForUI = topic("GoalForUI");
   public static final MessagerAPIFactory.Topic<List<String>> Parameters = topic("Parameters");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(DemoCoordinator).topic(apiFactory.createTypedTopicTheme(name));
   }
}
