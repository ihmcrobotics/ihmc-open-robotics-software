package us.ihmc.communication;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.ros2.ROS2Topic;

public class AutonomyAPI
{
   public static final String BEAVIOR_TREE_MODULE_NAME = "behavior_tree";

   public static final ROS2Topic<?> BEAVIOR_TREE_MODULE = ROS2Tools.IHMC_ROOT.withModule(BEAVIOR_TREE_MODULE_NAME);
   public static final ROS2IOTopicPair<BehaviorTreeStateMessage> BEAVIOR_TREE
         = new ROS2IOTopicPair<>(BEAVIOR_TREE_MODULE.withTypeName(BehaviorTreeStateMessage.class));
}
