package us.ihmc.behaviors.door;

import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;

public class DoorBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("DoorBehavior");
   private static final MessagerAPIFactory.CategoryTheme DoorTheme = apiFactory.createCategoryTheme("Door");

   public static final MessagerAPIFactory.Topic<Boolean> ReviewEnabled = topic("ReviewEnabled");
   public static final MessagerAPIFactory.Topic<Double> DistanceToDoor = topic("DistanceToDoor");
   public static final MessagerAPIFactory.Topic<Boolean> IsFacingDoor = topic("IsFacingDoor");
   public static final MessagerAPIFactory.Topic<MutablePair<DoorType, Pose3D>> DetectedDoorPose = topic("DetectedDoorPose");

   private static final <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(DoorTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static final MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
