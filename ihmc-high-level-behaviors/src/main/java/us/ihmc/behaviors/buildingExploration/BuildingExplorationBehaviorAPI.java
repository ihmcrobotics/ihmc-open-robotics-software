package us.ihmc.behaviors.buildingExploration;

import std_msgs.msg.dds.UInt16;
import us.ihmc.communication.DeprecatedAPIs;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.ros2.ROS2Topic;

public class BuildingExplorationBehaviorAPI
{
   private static final String MODULE_NAME = DeprecatedAPIs.BEHAVIOR_MODULE_NAME + "/building_exploration";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);

   public static final ROS2Topic<UInt16> MODE = BASE_TOPIC.withType(UInt16.class).withSuffix("mode");

   /**
    * Starts the behavior pursuing a goal if not already pursiung a goal.
    */
   public static final ROS2IOTopicPair<Pose3D> GOAL = new ROS2IOTopicPair<>(BASE_TOPIC.withType(Pose3D.class).withSuffix("goal"));
   public static final ROS2Topic<Pose3D> GOAL_COMMAND = GOAL.getCommandTopic();
   public static final ROS2Topic<Pose3D> GOAL_STATUS = GOAL.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "parameters");
   public static final ROS2Topic<std_msgs.msg.dds.String> LAST_TICKED_NODE
                                                        = BASE_TOPIC.withType(std_msgs.msg.dds.String.class).withSuffix("last_ticked_node");
}
