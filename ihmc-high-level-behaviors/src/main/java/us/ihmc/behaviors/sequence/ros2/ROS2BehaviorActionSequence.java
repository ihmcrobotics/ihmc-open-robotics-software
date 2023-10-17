package us.ihmc.behaviors.sequence.ros2;

import behavior_msgs.msg.dds.ActionSequenceUpdateMessage;
import behavior_msgs.msg.dds.ActionsExecutionStatusMessage;
import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Topic;

public class ROS2BehaviorActionSequence extends BehaviorActionSequence
{
   public static final ROS2Topic<?> ROOT_TOPIC = ROS2Tools.IHMC_ROOT.withRobot("behavior_action_sequence");
   public static final ROS2Topic<?> STATUS_TOPIC = ROOT_TOPIC.withOutput();
   public static final ROS2Topic<ActionsExecutionStatusMessage> ACTIONS_EXECUTION_STATUS
         = STATUS_TOPIC.withType(ActionsExecutionStatusMessage.class).withSuffix("execution_status");
   public static final ROS2Topic<BodyPartPoseStatusMessage> PELVIS_POSE_VARIATION_STATUS
         = STATUS_TOPIC.withType(BodyPartPoseStatusMessage.class).withSuffix("pelvis_pose_status");
   public static final ROS2Topic<BodyPartPoseStatusMessage> CHEST_POSE_STATUS
         = STATUS_TOPIC.withType(BodyPartPoseStatusMessage.class).withSuffix("chest_pose_status");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> RIGHT_HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("right_hand_pose_joint_angles");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> LEFT_HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("left_hand_pose_joint_angles");
   public static final ROS2Topic<std_msgs.msg.dds.String> EXECUTION_NEXT_INDEX_REJECTION_TOPIC
         = STATUS_TOPIC.withType(std_msgs.msg.dds.String.class).withSuffix("execution_next_index_rejection");
   public static final ROS2Topic<Int32> EXECUTION_NEXT_INDEX_STATUS_TOPIC = STATUS_TOPIC.withType(Int32.class).withSuffix("execution_next_index");
   public static final ROS2Topic<Bool> AUTOMATIC_EXECUTION_STATUS_TOPIC = STATUS_TOPIC.withType(Bool.class).withSuffix("automatic_execution");
   public static final ROS2Topic<ActionSequenceUpdateMessage> SEQUENCE_STATUS_TOPIC
         = STATUS_TOPIC.withType(ActionSequenceUpdateMessage.class).withSuffix("sequence_status");
   public static final ROS2Topic<?> COMMAND_TOPIC = ROOT_TOPIC.withInput();
   public static final ROS2Topic<Int32> EXECUTION_NEXT_INDEX_COMMAND_TOPIC = COMMAND_TOPIC.withType(Int32.class).withSuffix("execution_next_index");
   public static final ROS2Topic<Bool> AUTOMATIC_EXECUTION_COMMAND_TOPIC = COMMAND_TOPIC.withType(Bool.class).withSuffix("automatic_execution");
   public static final ROS2Topic<Empty> MANUALLY_EXECUTE_NEXT_ACTION_TOPIC = COMMAND_TOPIC.withType(Empty.class).withSuffix("manually_execute_next_action");
   public static final ROS2Topic<ActionSequenceUpdateMessage> SEQUENCE_COMMAND_TOPIC
         = COMMAND_TOPIC.withType(ActionSequenceUpdateMessage.class).withSuffix("sequence_update");

   private final ROS2ControllerHelper ros2;
   private final IHMCROS2Input<Empty> manuallyExecuteSubscription;
   private final IHMCROS2Input<Bool> automaticExecutionSubscription;
   private final IHMCROS2Input<Int32> executionNextIndexSubscription;
   private final IHMCROS2Input<ActionSequenceUpdateMessage> updateSubscription;
   public final Int32 executionNextIndexStatusMessage = new Int32();
   public final Bool automaticExecutionStatusMessage = new Bool();

   public ROS2BehaviorActionSequence(DRCRobotModel robotModel, ROS2ControllerHelper ros2, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(robotModel, referenceFrameLibrary);

      this.ros2 = ros2;

      updateSubscription = ros2.subscribe(ROS2BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC);
      manuallyExecuteSubscription = ros2.subscribe(ROS2BehaviorActionSequence.MANUALLY_EXECUTE_NEXT_ACTION_TOPIC);
      automaticExecutionSubscription = ros2.subscribe(ROS2BehaviorActionSequence.AUTOMATIC_EXECUTION_COMMAND_TOPIC);
      executionNextIndexSubscription = ros2.subscribe(ROS2BehaviorActionSequence.EXECUTION_NEXT_INDEX_COMMAND_TOPIC);
   }

   @Override
   public void update()
   {


      super.update();
   }
}
