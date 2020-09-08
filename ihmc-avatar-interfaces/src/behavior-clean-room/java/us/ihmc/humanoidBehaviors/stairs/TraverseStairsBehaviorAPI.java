package us.ihmc.humanoidBehaviors.stairs;

import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.ros2.ROS2Topic;

public class TraverseStairsBehaviorAPI
{
   private static final String ROS_TOPIC_QUALIFIER = "stairs";

   /**
    * Goal input, should be sent prior to sending "start"
    */
   public static final ROS2Topic<Pose3D> GOAL_INPUT = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Pose3D.class).withNaming(typeName -> typeName + "/goal");
   /**
    * Begins the behavior, should have received the goal input prior
    */
   public static final ROS2Topic<Empty> START = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class).withNaming(typeName -> typeName + "/start");
   /**
    * Stops the behavior, if a step is being executed it will finish that step then stand in place. Can be restarted through the start topic, the goal is persistent
    */
   public static final ROS2Topic<Empty> STOP = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class).withNaming(typeName -> typeName + "/stop");
   /**
    * Signals that the robot has reached the goal
    */
   public static final ROS2Topic<Empty> COMPLETED = ROS2Tools.BEHAVIOR_MODULE.withOutput().withType(Empty.class).withNaming(typeName -> typeName + "/completed");

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("TraverseStairsBehavior");
   private static final MessagerAPIFactory.CategoryTheme BehaviorTheme = apiFactory.createCategoryTheme("TraverseStairs");

   // TODO add api

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(BehaviorTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
