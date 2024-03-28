package us.ihmc.behaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.OldBehaviorAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Topic;

public class TraverseStairsBehaviorAPI
{
   private static final String ROS_TOPIC_QUALIFIER = "/stairs";

   /**
    * Goal input, should be sent prior to sending "start"
    */
   public static final ROS2Topic<Pose3D> GOAL_INPUT = OldBehaviorAPI.BEHAVIOR_MODULE.withInput().withType(Pose3D.class).withSuffix(ROS_TOPIC_QUALIFIER + "/goal");
   /**
    * Begins the behavior, should have received the goal input prior
    */
   public static final ROS2Topic<Empty> START = OldBehaviorAPI.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/start");
   /**
    * Stops the behavior, if a step is being executed it will finish that step then stand in place. Can be restarted through the start topic, the goal is persistent
    */
   public static final ROS2Topic<Empty> STOP = OldBehaviorAPI.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/stop");
   /**
    * Signals that the robot has reached the goal
    */
   public static final ROS2Topic<Empty> COMPLETED = OldBehaviorAPI.BEHAVIOR_MODULE.withOutput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/completed");
   /**
    * Planned steps to visualize
    */
   public static final ROS2Topic<FootstepDataListMessage> PLANNED_STEPS = OldBehaviorAPI.BEHAVIOR_MODULE.withOutput().withType(FootstepDataListMessage.class).withSuffix(ROS_TOPIC_QUALIFIER);
   /**
    * Signals the behavior to execute the steps, sent from the ui
    */
   public static final ROS2Topic<Empty> EXECUTE_STEPS = OldBehaviorAPI.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/execute_steps");
   /**
    * Signals the behavior to replan steps, sent from the ui
    */
   public static final ROS2Topic<Empty> REPLAN = OldBehaviorAPI.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/replan");
}
