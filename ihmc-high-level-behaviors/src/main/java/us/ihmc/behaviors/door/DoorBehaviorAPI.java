package us.ihmc.behaviors.door;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.ros2.ROS2Topic;

public class DoorBehaviorAPI
{
   private static final String MODULE_NAME = ROS2Tools.BEHAVIOR_MODULE_NAME + "/door_behavior";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("DoorBehavior");
   private static final MessagerAPIFactory.CategoryTheme DoorTheme = apiFactory.createCategoryTheme("Door");

   private static final <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(DoorTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static final MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
