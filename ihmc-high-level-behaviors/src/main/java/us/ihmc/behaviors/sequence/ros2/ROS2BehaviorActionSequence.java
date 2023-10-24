package us.ihmc.behaviors.sequence.ros2;

import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

/**
 * @deprecated Probably removing most of this
 */
public class ROS2BehaviorActionSequence
{
   public static final ROS2Topic<?> ROOT_TOPIC = ROS2Tools.IHMC_ROOT.withRobot("behavior_action_sequence");
   public static final ROS2Topic<?> STATUS_TOPIC = ROOT_TOPIC.withOutput();
   public static final ROS2Topic<BodyPartPoseStatusMessage> PELVIS_POSE_VARIATION_STATUS
         = STATUS_TOPIC.withType(BodyPartPoseStatusMessage.class).withSuffix("pelvis_pose_status");
   public static final ROS2Topic<BodyPartPoseStatusMessage> CHEST_POSE_STATUS
         = STATUS_TOPIC.withType(BodyPartPoseStatusMessage.class).withSuffix("chest_pose_status");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> RIGHT_HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("right_hand_pose_joint_angles");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> LEFT_HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("left_hand_pose_joint_angles");
}
