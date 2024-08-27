package us.ihmc.communication;

import behavior_msgs.msg.dds.AI2RCommandMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.ros2.ROS2Topic;

public final class AutonomyAPI
{
   public static final String BEAVIOR_TREE_MODULE_NAME = "behavior_tree";

   public static final ROS2Topic<?> BEAVIOR_TREE_MODULE = ROS2Tools.IHMC_ROOT.withModule(BEAVIOR_TREE_MODULE_NAME);
   public static final ROS2IOTopicPair<BehaviorTreeStateMessage> BEAVIOR_TREE
         = new ROS2IOTopicPair<>(BEAVIOR_TREE_MODULE.withTypeName(BehaviorTreeStateMessage.class));

   /** rt/ihmc/behavior_tree/ai2r_status */
   public static final ROS2Topic<AI2RStatusMessage> AI2R_STATUS = BEAVIOR_TREE_MODULE.withType(AI2RStatusMessage.class).withSuffix("ai2r_status");
   /** rt/ihmc/behavior_tree/ai2r_command */
   public static final ROS2Topic<AI2RCommandMessage> AI2R_COMMAND = BEAVIOR_TREE_MODULE.withType(AI2RCommandMessage.class).withSuffix("ai2r_command");
}
