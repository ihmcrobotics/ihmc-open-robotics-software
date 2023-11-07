package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionSequenceUpdateMessage extends Packet<ActionSequenceUpdateMessage> implements Settable<ActionSequenceUpdateMessage>, EpsilonComparable<ActionSequenceUpdateMessage>
{
   /**
            * Number of actions within the sequence
            */
   public int sequence_size_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage>  arm_joint_angles_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage>  chest_orientation_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage>  footstep_plan_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage>  sake_hand_command_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage>  hand_pose_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage>  hand_wrench_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage>  pelvis_height_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage>  wait_duration_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WalkActionStateMessage>  walk_actions_;

   public ActionSequenceUpdateMessage()
   {
      arm_joint_angles_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage> (200, new behavior_msgs.msg.dds.ArmJointAnglesActionStateMessagePubSubType());
      chest_orientation_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage> (200, new behavior_msgs.msg.dds.ChestOrientationActionStateMessagePubSubType());
      footstep_plan_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage> (200, new behavior_msgs.msg.dds.FootstepPlanActionStateMessagePubSubType());
      sake_hand_command_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage> (200, new behavior_msgs.msg.dds.SakeHandCommandActionStateMessagePubSubType());
      hand_pose_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage> (200, new behavior_msgs.msg.dds.HandPoseActionStateMessagePubSubType());
      hand_wrench_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage> (200, new behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType());
      pelvis_height_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage> (200, new behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessagePubSubType());
      wait_duration_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage> (200, new behavior_msgs.msg.dds.WaitDurationActionStateMessagePubSubType());
      walk_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WalkActionStateMessage> (200, new behavior_msgs.msg.dds.WalkActionStateMessagePubSubType());

   }

   public ActionSequenceUpdateMessage(ActionSequenceUpdateMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionSequenceUpdateMessage other)
   {
      sequence_size_ = other.sequence_size_;

      arm_joint_angles_actions_.set(other.arm_joint_angles_actions_);
      chest_orientation_actions_.set(other.chest_orientation_actions_);
      footstep_plan_actions_.set(other.footstep_plan_actions_);
      sake_hand_command_actions_.set(other.sake_hand_command_actions_);
      hand_pose_actions_.set(other.hand_pose_actions_);
      hand_wrench_actions_.set(other.hand_wrench_actions_);
      pelvis_height_actions_.set(other.pelvis_height_actions_);
      wait_duration_actions_.set(other.wait_duration_actions_);
      walk_actions_.set(other.walk_actions_);
   }

   /**
            * Number of actions within the sequence
            */
   public void setSequenceSize(int sequence_size)
   {
      sequence_size_ = sequence_size;
   }
   /**
            * Number of actions within the sequence
            */
   public int getSequenceSize()
   {
      return sequence_size_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage>  getArmJointAnglesActions()
   {
      return arm_joint_angles_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage>  getChestOrientationActions()
   {
      return chest_orientation_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage>  getFootstepPlanActions()
   {
      return footstep_plan_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage>  getSakeHandCommandActions()
   {
      return sake_hand_command_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage>  getHandPoseActions()
   {
      return hand_pose_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage>  getHandWrenchActions()
   {
      return hand_wrench_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage>  getPelvisHeightActions()
   {
      return pelvis_height_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage>  getWaitDurationActions()
   {
      return wait_duration_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WalkActionStateMessage>  getWalkActions()
   {
      return walk_actions_;
   }


   public static Supplier<ActionSequenceUpdateMessagePubSubType> getPubSubType()
   {
      return ActionSequenceUpdateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionSequenceUpdateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionSequenceUpdateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_size_, other.sequence_size_, epsilon)) return false;

      if (this.arm_joint_angles_actions_.size() != other.arm_joint_angles_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.arm_joint_angles_actions_.size(); i++)
         {  if (!this.arm_joint_angles_actions_.get(i).epsilonEquals(other.arm_joint_angles_actions_.get(i), epsilon)) return false; }
      }

      if (this.chest_orientation_actions_.size() != other.chest_orientation_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.chest_orientation_actions_.size(); i++)
         {  if (!this.chest_orientation_actions_.get(i).epsilonEquals(other.chest_orientation_actions_.get(i), epsilon)) return false; }
      }

      if (this.footstep_plan_actions_.size() != other.footstep_plan_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footstep_plan_actions_.size(); i++)
         {  if (!this.footstep_plan_actions_.get(i).epsilonEquals(other.footstep_plan_actions_.get(i), epsilon)) return false; }
      }

      if (this.sake_hand_command_actions_.size() != other.sake_hand_command_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.sake_hand_command_actions_.size(); i++)
         {  if (!this.sake_hand_command_actions_.get(i).epsilonEquals(other.sake_hand_command_actions_.get(i), epsilon)) return false; }
      }

      if (this.hand_pose_actions_.size() != other.hand_pose_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.hand_pose_actions_.size(); i++)
         {  if (!this.hand_pose_actions_.get(i).epsilonEquals(other.hand_pose_actions_.get(i), epsilon)) return false; }
      }

      if (this.hand_wrench_actions_.size() != other.hand_wrench_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.hand_wrench_actions_.size(); i++)
         {  if (!this.hand_wrench_actions_.get(i).epsilonEquals(other.hand_wrench_actions_.get(i), epsilon)) return false; }
      }

      if (this.pelvis_height_actions_.size() != other.pelvis_height_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.pelvis_height_actions_.size(); i++)
         {  if (!this.pelvis_height_actions_.get(i).epsilonEquals(other.pelvis_height_actions_.get(i), epsilon)) return false; }
      }

      if (this.wait_duration_actions_.size() != other.wait_duration_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.wait_duration_actions_.size(); i++)
         {  if (!this.wait_duration_actions_.get(i).epsilonEquals(other.wait_duration_actions_.get(i), epsilon)) return false; }
      }

      if (this.walk_actions_.size() != other.walk_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.walk_actions_.size(); i++)
         {  if (!this.walk_actions_.get(i).epsilonEquals(other.walk_actions_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionSequenceUpdateMessage)) return false;

      ActionSequenceUpdateMessage otherMyClass = (ActionSequenceUpdateMessage) other;

      if(this.sequence_size_ != otherMyClass.sequence_size_) return false;

      if (!this.arm_joint_angles_actions_.equals(otherMyClass.arm_joint_angles_actions_)) return false;
      if (!this.chest_orientation_actions_.equals(otherMyClass.chest_orientation_actions_)) return false;
      if (!this.footstep_plan_actions_.equals(otherMyClass.footstep_plan_actions_)) return false;
      if (!this.sake_hand_command_actions_.equals(otherMyClass.sake_hand_command_actions_)) return false;
      if (!this.hand_pose_actions_.equals(otherMyClass.hand_pose_actions_)) return false;
      if (!this.hand_wrench_actions_.equals(otherMyClass.hand_wrench_actions_)) return false;
      if (!this.pelvis_height_actions_.equals(otherMyClass.pelvis_height_actions_)) return false;
      if (!this.wait_duration_actions_.equals(otherMyClass.wait_duration_actions_)) return false;
      if (!this.walk_actions_.equals(otherMyClass.walk_actions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionSequenceUpdateMessage {");
      builder.append("sequence_size=");
      builder.append(this.sequence_size_);      builder.append(", ");
      builder.append("arm_joint_angles_actions=");
      builder.append(this.arm_joint_angles_actions_);      builder.append(", ");
      builder.append("chest_orientation_actions=");
      builder.append(this.chest_orientation_actions_);      builder.append(", ");
      builder.append("footstep_plan_actions=");
      builder.append(this.footstep_plan_actions_);      builder.append(", ");
      builder.append("sake_hand_command_actions=");
      builder.append(this.sake_hand_command_actions_);      builder.append(", ");
      builder.append("hand_pose_actions=");
      builder.append(this.hand_pose_actions_);      builder.append(", ");
      builder.append("hand_wrench_actions=");
      builder.append(this.hand_wrench_actions_);      builder.append(", ");
      builder.append("pelvis_height_actions=");
      builder.append(this.pelvis_height_actions_);      builder.append(", ");
      builder.append("wait_duration_actions=");
      builder.append(this.wait_duration_actions_);      builder.append(", ");
      builder.append("walk_actions=");
      builder.append(this.walk_actions_);
      builder.append("}");
      return builder.toString();
   }
}
