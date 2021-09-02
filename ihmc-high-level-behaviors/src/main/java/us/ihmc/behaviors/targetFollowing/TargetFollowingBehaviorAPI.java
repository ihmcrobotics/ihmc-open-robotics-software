package us.ihmc.behaviors.targetFollowing;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;

import java.util.List;

public class TargetFollowingBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("TargetFollowingRoot");
   private static final MessagerAPIFactory.CategoryTheme TargetFollower = apiFactory.createCategoryTheme("TargetFollower");

   public static final MessagerAPIFactory.Topic<Pose3D> Goal = topic("Goal");
   public static final MessagerAPIFactory.Topic<List<String>> Parameters = topic("Parameters");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(TargetFollower).topic(apiFactory.createTypedTopicTheme(name));
   }
}
