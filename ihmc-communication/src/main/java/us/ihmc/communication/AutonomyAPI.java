package us.ihmc.communication;

import behavior_msgs.msg.dds.AI2RCommandMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import behavior_msgs.msg.dds.BehaviorTreeYoDataMessage;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public final class AutonomyAPI
{
   public static final String BEHAVIOR_TREE_MODULE_NAME = "behavior_tree";

   public static final ROS2Topic<?> BEHAVIOR_TREE_MODULE = ROS2Tools.IHMC_ROOT.withModule(BEHAVIOR_TREE_MODULE_NAME)
                                                                              .withQoS(ROS2QosProfile.RELIABLE());
   public static final ROS2IOTopicPair<BehaviorTreeStateMessage> BEHAVIOR_TREE
         = new ROS2IOTopicPair<>(BEHAVIOR_TREE_MODULE.withTypeName(BehaviorTreeStateMessage.class));

   /** Used to send data to the controller for YoVariablization. */
   public static final ROS2Topic<BehaviorTreeYoDataMessage> BEHAVIOR_YO_DATA = BEHAVIOR_TREE_MODULE.withQoS(ROS2QosProfile.BEST_EFFORT())
                                                                                                   .withTypeName(BehaviorTreeYoDataMessage.class);

   /** rt/ihmc/behavior_tree/ai2r_status */
   public static final ROS2Topic<AI2RStatusMessage> AI2R_STATUS = BEHAVIOR_TREE_MODULE.withType(AI2RStatusMessage.class).withSuffix("ai2r_status");
   /** rt/ihmc/behavior_tree/ai2r_command */
   public static final ROS2Topic<AI2RCommandMessage> AI2R_COMMAND = BEHAVIOR_TREE_MODULE.withType(AI2RCommandMessage.class).withSuffix("ai2r_command");
}
