package us.ihmc.communication;

import behavior_msgs.AI2RCommandMessage;
import behavior_msgs.AI2RStatusMessage;
import behavior_msgs.BehaviorTreeStateMessage;
import behavior_msgs.BehaviorTreeYoDataMessage;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;

public final class AutonomyAPI
{
   public static final String BEHAVIOR_TREE_MODULE_NAME = "behavior_tree";

   public static final HumanoidROS2Topic<?> BEHAVIOR_TREE_MODULE = HumanoidROS2Topic.IHMC_ROOT.withModule(BEHAVIOR_TREE_MODULE_NAME);
   public static final ROS2IOTopicPair<BehaviorTreeStateMessage> BEHAVIOR_TREE
         = new ROS2IOTopicPair<>(BEHAVIOR_TREE_MODULE.withTypeName(BehaviorTreeStateMessage.class));

   /** Used to send data to the controller for YoVariablization. */
   public static final ROS2Topic<BehaviorTreeYoDataMessage> BEHAVIOR_YO_DATA = BEHAVIOR_TREE_MODULE.withTypeName(BehaviorTreeYoDataMessage.class)
                                                                                                   .withQoS(ROS2QoSProfile.BEST_EFFORT);

   /** rt/ihmc/behavior_tree/ai2r_status */
   public static final ROS2Topic<AI2RStatusMessage> AI2R_STATUS = BEHAVIOR_TREE_MODULE.withType(AI2RStatusMessage.class).withSuffix("ai2r_status");
   /** rt/ihmc/behavior_tree/ai2r_command */
   public static final ROS2Topic<AI2RCommandMessage> AI2R_COMMAND = BEHAVIOR_TREE_MODULE.withType(AI2RCommandMessage.class).withSuffix("ai2r_command");
}
