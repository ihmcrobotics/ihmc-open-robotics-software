package us.ihmc.behaviors.targetFollowing;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;

import java.util.List;

public class TargetFollowingBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("TargetFollowingRoot");
   private static final MessagerAPIFactory.CategoryTheme TargetFollower = apiFactory.createCategoryTheme("TargetFollower");

   public static final MessagerAPIFactory.Topic<Pose3D> TargetPose = topic("TargetPose");
   public static final MessagerAPIFactory.Topic<Pose3D> TargetApproachPose = topic("TargetApproachPose");
   public static final MessagerAPIFactory.Topic<List<String>> TargetFollowingParameters = topic("TargetFollowingParameters");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(TargetFollower).topic(apiFactory.createTypedTopicTheme(name));
   }
}
